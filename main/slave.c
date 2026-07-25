#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "camera.h"

#define TLOG(fmt, ...) do { uint64_t _ms = esp_timer_get_time() / 1000; \
    printf("[%5llu.%03llu] " fmt, _ms / 1000, _ms % 1000, ##__VA_ARGS__); } while(0)

// Direkter TRNG-Register-Zugriff (identisch mit Master)
#define RNG_REG       (*((volatile uint32_t *)0x501101A4UL))
#define TRNG_PER_RUN  200000
#define SEGMENT_BITS  200
#define NUM_SEGMENTS  ((TRNG_PER_RUN * 32) / SEGMENT_BITS)   // 32000

// Segments per run — must mirror the master's sensor.c exactly, or the two
// nodes measure over different integration lengths.
#define TRNG_SEGMENTS NUM_SEGMENTS   // 6.4 Mbit/run
#define CAM_SEGMENTS  8000           // 1.6 Mbit/run ≈ 0.47 s at ~3.4 Mbit/s

// This node's own entropy source. It must be *independent* of the master's:
// each node has its own camera, never a shared one, or the ÷√2 combination
// would be counting one measurement twice.
typedef enum { SRC_TRNG = 0, SRC_CAM = 1 } SlaveSource;
static volatile SlaveSource g_src = SRC_TRNG;
static volatile bool        g_src_fell_back = false;

// UART1 for master communication: TX=GPIO14, RX=GPIO15
// Wiring: Master-GPIO14 → Slave-GPIO15, Slave-GPIO14 → Master-GPIO15, GND-GND
#define UART_PORT    UART_NUM_1
#define UART_TX_PIN  14
#define UART_RX_PIN  15
#define UART_BAUD    460800   // must match SLAVE_BAUD in elotto/main/sensor.c

static double        g_baseline_mean = 0.0;
static volatile bool g_abort         = false;
static bool          g_fallback_logged = false;

// Dump camera health to the console. Called on the first fallback (so the
// reason a run stopped being camera-sourced is recorded at the moment it
// happens) and on demand via the 'D' command.
static void log_camera_stats(const char *why)
{
    camera_stats_t cs;
    camera_get_stats(&cs);
    // mean_pixel is the light-leak check (must sit at the black floor) and the
    // autocorrelations reveal flicker: a mains-frequency leak would put a global
    // frame-to-frame intensity swing into every diff, correlating pixels that
    // should be independent. Both are needed to tell a light problem apart from
    // sensor/ISP variation.
    TLOG("cam[%s] ready=%d pairs=%llu mbit_s=%.3f stalls=%lu waits=%lu drops=%lu "
         "stuck=%lu bias=%.6f sigma=%.4f mean_px=%.2f zero_diff=%.4f "
         "r1=%.4f r2=%.4f r3=%.4f r4=%.4f\n",
         why, (int)cs.ready, (unsigned long long)cs.frame_pairs, cs.mbit_per_sec,
         (unsigned long)cs.stalls, (unsigned long)cs.consumer_waits,
         (unsigned long)cs.ring_drops, (unsigned long)cs.stuck_frame_count,
         cs.bias, cs.sigma, cs.mean_pixel_level, cs.zero_diff_frac,
         cs.autocorr_lag[0], cs.autocorr_lag[1], cs.autocorr_lag[2], cs.autocorr_lag[3]);
}

static inline uint32_t fast_rng(void) { return RNG_REG; }

// Mirrors the master's noise_word(): on a camera stall, latch to the TRNG so
// every later word does not re-pay the stall timeout, and remember that this
// run is no longer camera-sourced so the master can be told.
static inline uint32_t noise_word(void)
{
    if (g_src == SRC_CAM) {
        uint32_t w;
        if (camera_read_word(&w)) return w;
        g_src = SRC_TRNG;
        g_src_fell_back = true;
    }
    return RNG_REG;
}

static double gcp_zscore_raw(void)
{
    const bool cam  = (g_src == SRC_CAM);
    const int  nseg = cam ? CAM_SEGMENTS : TRNG_SEGMENTS;
    // Abort latency, not pacing: the camera run is 4x shorter in segments, so
    // poll 4x more often to keep the response time comparable.
    const int  poll = cam ? 2000 : 8000;

    double z_sum = 0.0;
    for (int seg = 0; seg < nseg; seg++) {
        int ones = __builtin_popcount(noise_word())
                 + __builtin_popcount(noise_word())
                 + __builtin_popcount(noise_word())
                 + __builtin_popcount(noise_word())
                 + __builtin_popcount(noise_word())
                 + __builtin_popcount(noise_word())
                 + __builtin_popcount(noise_word() & 0xFF);
        z_sum += (ones - 100.0) / 7.07106781;
        if (seg % poll == 0) {   // TRNG: 4 yields/run (~40 ms), matching master
            vTaskDelay(1);
            // Abort-Check via UART (non-blocking)
            uint8_t ch = 0;
            if (uart_read_bytes(UART_PORT, &ch, 1, 0) > 0 && ch == 'A')
                g_abort = true;
            if (g_abort) return 0.0;
        }
    }
    return z_sum / sqrt((double)nseg);
}

void app_main(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 512, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // This task IS the entropy consumer (the measurement loop runs inline in the
    // command handlers below). IDF hardcodes app_main's priority to 1, which is
    // *below* the camera extraction task — the producer would then starve the
    // consumer and a run would take 5.1 s instead of 0.47 s. Outrank it, the way
    // the master's elotto_task (priority 5) already does.
    vTaskPrioritySet(NULL, ELOTTO_CAM_TASK_PRIO + 1);

    // Camera entropy (this node's own OV5647 — never shared with the master).
    // Non-fatal: without it the slave still measures on its TRNG, which is the
    // pre-camera behaviour, and reports 'T' so the master can flag the mix.
    esp_err_t cam_ret = camera_init();
    if (cam_ret == ESP_OK) {
        // Wait briefly for the first frame pair so the ring has bits before the
        // master's first M command arrives.
        for (int i = 0; i < 100 && !camera_is_ready(); i++) vTaskDelay(pdMS_TO_TICKS(50));
    }
    g_src = camera_is_ready() ? SRC_CAM : SRC_TRNG;
    if (g_src == SRC_CAM) {
        vTaskDelay(pdMS_TO_TICKS(3000));   // let stats accumulate before reporting
        log_camera_stats("idle-boot");     // light-leak baseline, no measurement load
    }

    vTaskDelay(pdMS_TO_TICKS(200));   // Startup-Rauschen abklingen lassen
    uart_flush_input(UART_PORT);
    TLOG("GCP-Slave ready  UART1 TX=GPIO%d RX=GPIO%d  %d Baud  source=%s\n",
         UART_TX_PIN, UART_RX_PIN, UART_BAUD,
         (g_src == SRC_CAM) ? "camera" : "TRNG");
    if (cam_ret != ESP_OK)
        TLOG("camera_init: %s -- measuring on TRNG\n", esp_err_to_name(cam_ret));

    // Protocol:
    //   P\n       → OK\n              (Ping)
    //   B<n>\n    → OK\n              (Baseline, n runs)
    //   M\n       → Z:<float>,<C|T>\n (Measure; C = camera, T = TRNG fallback)
    //   D\n       → D:ready,bias,sigma,mbit_s,stalls,stuck,<C|T>\n (camera diag)
    //   A\n       → OK\n              (Abort)

    char line[64];
    int  pos = 0;

    while (1) {
        uint8_t ch;
        if (uart_read_bytes(UART_PORT, &ch, 1, portMAX_DELAY) <= 0) continue;
        if (ch != '\n') {
            if (ch == 0x00 || ch >= 0x80) { pos = 0; continue; }   // break / boot noise → verwerfen
            if (pos < (int)sizeof(line) - 1) line[pos++] = (char)ch;
            continue;
        }
        line[pos] = '\0';
        pos = 0;

        if (line[0] == 'P') {
            uart_write_bytes(UART_PORT, "OK\n", 3);
            TLOG("Ping received -> OK sent\n");

        } else if (line[0] == 'B') {
            int cnt = atoi(line + 1);
            if (cnt <= 0 || cnt > 5000) cnt = 100;
            g_abort = false;
            double bsum = 0.0;
            int    done = 0;
            for (; done < cnt && !g_abort; done++)
                bsum += gcp_zscore_raw();
            g_baseline_mean = g_abort ? 0.0 : bsum / cnt;
            uart_write_bytes(UART_PORT, "OK\n", 3);
            TLOG("Baseline done: mean=%.4f (%d/%d runs) source=%s\n",
                 g_baseline_mean, done, cnt,
                 (g_src == SRC_CAM) ? "camera" : "TRNG");
            log_camera_stats("after-baseline");

        } else if (line[0] == 'M') {
            g_abort = false;
            double z = gcp_zscore_raw() - g_baseline_mean;
            // Reply carries the source this run actually used: 'C' camera,
            // 'T' TRNG. Appended after the float so atof() still parses it —
            // an older master reading only the number stays compatible.
            char resp[40];
            int  len = snprintf(resp, sizeof(resp), "Z:%.6f,%c\n", z,
                                (g_src == SRC_CAM) ? 'C' : 'T');
            uart_write_bytes(UART_PORT, resp, len);
            if (g_src_fell_back && !g_fallback_logged) {
                g_fallback_logged = true;
                log_camera_stats("FALLBACK->TRNG");
            }

        } else if (line[0] == 'D') {
            // Camera diagnostics, so the master can surface slave-side health
            // instead of only seeing 'C'/'T' on each measurement.
            camera_stats_t cs;
            camera_get_stats(&cs);
            char r[128];
            int  len = snprintf(r, sizeof(r),
                                "D:%d,%.6f,%.4f,%.3f,%lu,%lu,%c\n",
                                (int)cs.ready, cs.bias, cs.sigma, cs.mbit_per_sec,
                                (unsigned long)cs.stalls, (unsigned long)cs.stuck_frame_count,
                                (g_src == SRC_CAM) ? 'C' : 'T');
            uart_write_bytes(UART_PORT, r, len);
            log_camera_stats("on-demand");

        } else if (line[0] == 'A') {
            g_abort = true;
            uart_write_bytes(UART_PORT, "OK\n", 3);
        } else if (pos == 0 && line[0] != '\0') {
            TLOG("Unknown command: '%c' (0x%02X)\n", line[0], (uint8_t)line[0]);
        }
    }
}
