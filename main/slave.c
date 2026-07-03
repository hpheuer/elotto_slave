#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"

#define TLOG(fmt, ...) do { uint64_t _ms = esp_timer_get_time() / 1000; \
    printf("[%5llu.%03llu] " fmt, _ms / 1000, _ms % 1000, ##__VA_ARGS__); } while(0)

// Direkter TRNG-Register-Zugriff (identisch mit Master)
#define RNG_REG       (*((volatile uint32_t *)0x501101A4UL))
#define TRNG_PER_RUN  200000
#define SEGMENT_BITS  200
#define NUM_SEGMENTS  ((TRNG_PER_RUN * 32) / SEGMENT_BITS)   // 32000

// UART1 for master communication: TX=GPIO14, RX=GPIO15
// Wiring: Master-GPIO14 → Slave-GPIO15, Slave-GPIO14 → Master-GPIO15, GND-GND
#define UART_PORT    UART_NUM_1
#define UART_TX_PIN  14
#define UART_RX_PIN  15
#define UART_BAUD    460800   // must match SLAVE_BAUD in elotto/main/sensor.c

static double        g_baseline_mean = 0.0;
static volatile bool g_abort         = false;

static inline uint32_t fast_rng(void) { return RNG_REG; }

static double gcp_zscore_raw(void)
{
    double z_sum = 0.0;
    for (int seg = 0; seg < NUM_SEGMENTS; seg++) {
        int ones = __builtin_popcount(fast_rng())
                 + __builtin_popcount(fast_rng())
                 + __builtin_popcount(fast_rng())
                 + __builtin_popcount(fast_rng())
                 + __builtin_popcount(fast_rng())
                 + __builtin_popcount(fast_rng())
                 + __builtin_popcount(fast_rng() & 0xFF);
        z_sum += (ones - 100.0) / 7.07106781;
        if (seg % 8000 == 0) {   // 4 yields/run (~40 ms) — must match master pace
            vTaskDelay(1);
            // Abort-Check via UART (non-blocking)
            uint8_t ch = 0;
            if (uart_read_bytes(UART_PORT, &ch, 1, 0) > 0 && ch == 'A')
                g_abort = true;
            if (g_abort) return 0.0;
        }
    }
    return z_sum / sqrt((double)NUM_SEGMENTS);
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

    vTaskDelay(pdMS_TO_TICKS(200));   // Startup-Rauschen abklingen lassen
    uart_flush_input(UART_PORT);
    TLOG("GCP-Slave ready  UART1 TX=GPIO%d RX=GPIO%d  %d Baud\n",
         UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    // Protocol:
    //   P\n       → OK\n          (Ping)
    //   B<n>\n    → OK\n          (Baseline, n runs)
    //   M\n       → Z:<float>\n   (Measure)
    //   A\n       → OK\n          (Abort)

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
            TLOG("Baseline done: mean=%.4f (%d/%d runs)\n",
                 g_baseline_mean, done, cnt);

        } else if (line[0] == 'M') {
            g_abort = false;
            double z = gcp_zscore_raw() - g_baseline_mean;
            char resp[32];
            int  len = snprintf(resp, sizeof(resp), "Z:%.6f\n", z);
            uart_write_bytes(UART_PORT, resp, len);

        } else if (line[0] == 'A') {
            g_abort = true;
            uart_write_bytes(UART_PORT, "OK\n", 3);
        } else if (pos == 0 && line[0] != '\0') {
            TLOG("Unknown command: '%c' (0x%02X)\n", line[0], (uint8_t)line[0]);
        }
    }
}
