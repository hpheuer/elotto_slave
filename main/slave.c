/* GCP measurement slave — Phase C of docs/PLAN_NETWORK.md.
 *
 * WHAT CHANGED IN PHASE C: the transport. This node used to be reachable only
 * over a UART1 crossover to the master (TX=GPIO14 / RX=GPIO15, 460800 baud);
 * it is now an Ethernet node that
 *   - answers the same 'P'/'B'/'M'/'D'/'A' commands over UDP, and
 *   - serves HTTP, so it can be updated over the wire like every other node.
 *
 * The HTTP server is not a convenience. With rollback armed, an app that cannot
 * be reached over the network is rolled back by design on the next boot
 * (PLAN_NETWORK §3a), so a slave without a webserver could not be installed at
 * all. That is exactly why the slave has been running the recovery updater
 * since Phase A.
 *
 * The measurement code below — noise_word(), gcp_zscore_raw(), the segment
 * counts, the 'C'/'T' source tag — is untouched, deliberately: Phase C is an
 * A/B of the transport, so anything that could move pair_r or sigma must stay
 * byte-identical to the UART era.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "camera.h"
#include "elotto_link.h"
#include "elotto_ota.h"

static const char *TAG = "slave";

#define TLOG(fmt, ...) do { uint64_t _ms = esp_timer_get_time() / 1000; \
    printf("[%5llu.%03llu] " fmt, _ms / 1000, _ms % 1000, ##__VA_ARGS__); } while(0)

// Direkter TRNG-Register-Zugriff (identisch mit Master)
#define RNG_REG       (*((volatile uint32_t *)0x501101A4UL))
#define TRNG_PER_RUN  200000
#define SEGMENT_BITS  200
#define NUM_SEGMENTS  ((TRNG_PER_RUN * 32) / SEGMENT_BITS)   // 32000

// Segments per run come FROM THE MASTER now (PLAN_4NODE Phase 5): 'B' and 'M'
// carry the count, because Phase 5 made run length phase-dependent (a scoring
// run holds its candidate number twice as long as a measurement run holds a
// draw) and a duplicated constant on each side would let the nodes integrate
// different windows while every published number stayed plausible.
//
// These remain only as the fallback for a master that sends no count — i.e. a
// pre-Phase-5 image. A fallback is not a second source of truth: if it is ever
// used the console says so.
#define TRNG_SEGMENTS NUM_SEGMENTS   // 6.4 Mbit/run
#define CAM_SEGMENTS  8000           // 1.6 Mbit/run ≈ 0.47 s at ~3.4 Mbit/s
#define SEG_MIN        100
#define SEG_MAX     200000

// This node's own entropy source. It must be *independent* of the master's:
// each node has its own camera, never a shared one, or the ÷√2 combination
// would be counting one measurement twice.
typedef enum { SRC_TRNG = 0, SRC_CAM = 1 } SlaveSource;
static volatile SlaveSource g_src = SRC_TRNG;
static volatile bool        g_src_fell_back = false;

/* The calibration sweep's table (~1.2 KB). PSRAM and allocated once: the camera
 * already makes PSRAM mandatory, and a struct this size does not belong on the
 * link task's stack. NULL means the allocation failed, which the 'K' handler
 * reports rather than papering over. */
static camera_cal_t *s_cal;

static double        g_baseline_mean = 0.0;
static volatile bool g_abort         = false;
static bool          g_fallback_logged = false;
static volatile bool g_measuring     = false;   // refuses OTA mid-measurement

/* ── Ethernet (Waveshare ESP32-P4-ETH, IP101GRI over RMII) ─────────────
 * Same pins and bring-up as the master's elotto.c and the updater's
 * ota_main.c — proven on this exact board, unlike examples/ethernet/basic. */
#define ETH_MDC_GPIO      31
#define ETH_MDIO_GPIO     52
#define ETH_PHY_RST_GPIO  51
#define ETH_PHY_ADDR       1
#define ETH_GOT_IP_BIT    BIT0

static EventGroupHandle_t s_eth_events;

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_eth_events, ETH_GOT_IP_BIT);
    }
}

static void ethernet_init(void)
{
    esp_netif_config_t ncfg  = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t       *netif = esp_netif_new(&ncfg);

    eth_mac_config_t        mac_cfg  = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num  = ETH_MDC_GPIO;
    emac_cfg.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    emac_cfg.interface         = EMAC_DATA_INTERFACE_RMII;
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr       = ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = ETH_PHY_RST_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_cfg);

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_hdl = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &eth_hdl));
    esp_netif_attach(netif, esp_eth_new_netif_glue(eth_hdl));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_ETH_GOT_IP, on_ip_event, NULL, NULL));
    esp_eth_start(eth_hdl);
}

/* ── Camera health logging ────────────────────────────────────────────── */

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

/* ── UDP command link ─────────────────────────────────────────────────── */

static int s_sock = -1;

// One-entry reply cache. The master resends a timed-out command under the SAME
// sequence number, so a repeat of something already answered is served from
// here instead of re-measuring: a lost *reply* then costs a round trip, not a
// second 0.5 s run whose bits nobody asked for.
static uint32_t s_last_seq;
static bool     s_have_last = false;
static char     s_last_reply[ELOTTO_LINK_MAX];

static void link_reply(const struct sockaddr_in *to, uint32_t seq, const char *payload)
{
    char msg[ELOTTO_LINK_MAX];
    int  n = elotto_link_pack(msg, sizeof(msg), seq, payload);
    if (n <= 0) return;
    sendto(s_sock, msg, n, 0, (const struct sockaddr *)to, sizeof(*to));
    s_last_seq  = seq;
    s_have_last = true;
    snprintf(s_last_reply, sizeof(s_last_reply), "%s", payload);
}

/* Called from inside a run, where the command loop is not reading the socket.
 * PEEKs first and consumes only an 'A': anything else — typically the master's
 * resend of the command being executed right now — has to stay queued so the
 * command loop still sees it and can answer from the cache. */
static void link_poll_abort(void)
{
    char buf[ELOTTO_LINK_MAX];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    int n = recvfrom(s_sock, buf, sizeof(buf) - 1, MSG_PEEK | MSG_DONTWAIT,
                     (struct sockaddr *)&from, &fl);
    if (n <= 0) return;
    buf[n] = '\0';

    uint32_t seq;
    char    *payload;
    if (!elotto_link_parse(buf, &seq, &payload) || payload[0] != 'A') return;

    fl = sizeof(from);
    recvfrom(s_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
    g_abort = true;
    link_reply(&from, seq, "OK");
}

/* Drop whatever is queued. Runs when a session starts ('B'), mirroring the
 * uart_flush_input() the UART path did there: a leftover 'A' from the session
 * that was aborted would otherwise be consumed by the first baseline run and
 * abort this one too. */
static void link_drain(void)
{
    char buf[ELOTTO_LINK_MAX];
    struct sockaddr_in from;
    socklen_t fl;
    for (;;) {
        fl = sizeof(from);
        if (recvfrom(s_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT,
                     (struct sockaddr *)&from, &fl) <= 0) return;
    }
}

/* ── Measurement (unchanged from the UART era — see file header) ──────── */

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

/* Segment count for this run: what the master asked for, or this node's own
 * default if the command carried none. `arg` points just past the command
 * letter (and past the ',' for 'B'). */
static int seg_from_cmd(const char *arg)
{
    int nseg = arg ? atoi(arg) : 0;
    if (nseg >= SEG_MIN && nseg <= SEG_MAX) return nseg;
    nseg = (g_src == SRC_CAM) ? CAM_SEGMENTS : TRNG_SEGMENTS;
    TLOG("no segment count on the wire -- falling back to %d (pre-Phase-5 master?)\n",
         nseg);
    return nseg;
}

static double gcp_zscore_raw(int nseg)
{
    // Abort latency, not pacing: keep the yield cadence a fixed fraction of the
    // run so it stays matched to the master's at every run length.
    const int poll = nseg / 4 + 1;

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
        if (seg % poll == 0) {   // 4 yields/run, matching the master's cadence
            vTaskDelay(1);
            link_poll_abort();
            if (g_abort) return 0.0;
        }
    }
    return z_sum / sqrt((double)nseg);
}

/* ── HTTP: own /diag, plus the shared update endpoints ────────────────── */

static bool slave_busy(void) { return g_measuring; }

static esp_err_t diag_handler(httpd_req_t *req)
{
    camera_stats_t cs;
    camera_get_stats(&cs);
    char buf[768];
    int  pos = snprintf(buf, sizeof(buf),
        "{\"role\":\"slave\",\"src\":\"%s\",\"measuring\":%s,\"baseline_mean\":%.4f,",
        (g_src == SRC_CAM) ? "cam" : "trng", g_measuring ? "true" : "false",
        g_baseline_mean);
    pos += elotto_ota_status_json(buf + pos, sizeof(buf) - pos);
    snprintf(buf + pos, sizeof(buf) - pos,
        ",\"cam\":{\"ready\":%s,\"frame_pairs\":%llu,\"bias\":%.6f,\"sigma\":%.4f,"
        "\"mean_pixel\":%.2f,\"mbit_s\":%.3f,\"zero_diff\":%.4f,\"stuck_frames\":%lu,"
        "\"drops\":%lu,\"waits\":%lu,\"stalls\":%lu,"
        "\"autocorr\":[%.4f,%.4f,%.4f,%.4f]}}",
        cs.ready ? "true" : "false", (unsigned long long)cs.frame_pairs,
        cs.bias, cs.sigma, cs.mean_pixel_level, cs.mbit_per_sec, cs.zero_diff_frac,
        (unsigned long)cs.stuck_frame_count, (unsigned long)cs.ring_drops,
        (unsigned long)cs.consumer_waits, (unsigned long)cs.stalls,
        cs.autocorr_lag[0], cs.autocorr_lag[1], cs.autocorr_lag[2], cs.autocorr_lag[3]);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!DOCTYPE html><meta charset=utf-8><title>elotto slave</title>"
        "<style>body{font-family:sans-serif;background:#0a2e0a;color:#eee;padding:24px}"
        "code{background:#00000055;padding:2px 6px;border-radius:4px}a{color:#90ee90}</style>"
        "<h2>elotto GCP slave</h2>"
        "<p>Measures on its own OV5647; triggered by the master over UDP broadcast "
        "on port 5000 (docs/PLAN_NETWORK.md Phase C).</p>"
        "<ul>"
        "<li><a href='/diag'>/diag</a> &mdash; camera health, source, firmware</li>"
        "<li><a href='/otainfo'>/otainfo</a> &mdash; image version / slot / state</li>"
        "<li><code>POST /update</code> &mdash; refused with 409 while measuring</li>"
        "</ul>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, page);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size        = 8192;
    cfg.max_uri_handlers  = 10;
    cfg.recv_wait_timeout = 20;   /* /update streams a multi-hundred-KB body */
    cfg.send_wait_timeout = 20;
    cfg.lru_purge_enable  = true;

    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));
    static const httpd_uri_t root = {"/",     HTTP_GET, root_handler, NULL};
    static const httpd_uri_t diag = {"/diag", HTTP_GET, diag_handler, NULL};
    httpd_register_uri_handler(srv, &root);
    httpd_register_uri_handler(srv, &diag);
    return srv;
}

/* ── Command loop ─────────────────────────────────────────────────────────
 * Protocol, unchanged from the UART era; only the framing around it is new
 * (see components/elotto_link/include/elotto_link.h):
 *   P        → OK                  (discovery, was the wired ping)
 *   K<ms>    → OK:exp,gain,fold,bias,mbit_s,<G|U>   (calibrate the camera)
 *   B<n>,<s> → OK                  (baseline, n runs of s segments each)
 *   M<s>     → Z:<float>,<C|T>     (measure s segments; C = camera, T = TRNG)
 *   D        → D:ready,bias,sigma,mbit_s,stalls,stuck,<C|T>
 *   A        → OK                  (abort)
 *
 * The segment counts on 'B'/'M' are the Phase 5 addition; 'K' is Task 1 of
 * docs/PLAN.md. Everything else is unchanged from the UART era.
 */

/* Abort poll for the calibration sweep. The sweep blocks this task for tens of
 * seconds, and it is the ONLY reader of the socket — without pumping it here an
 * 'A' would sit unread until the sweep finished, which is exactly the case the
 * abort exists for. Same peek-and-consume-only-'A' rule as inside a run, so the
 * master's resend of the 'K' being executed stays queued for the reply cache. */
static bool cal_abort_cb(void)
{
    link_poll_abort();
    return g_abort;
}
static void link_task(void *arg)
{
    // This task IS the entropy consumer. The camera extraction task is
    // CPU-hungry, so a consumer at or below its priority starves the producer
    // and a run takes 5.1 s instead of 0.47 s (see camera.h).
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) { ESP_LOGE(TAG, "socket() failed"); vTaskDelete(NULL); return; }
    struct sockaddr_in me = {
        .sin_family      = AF_INET,
        .sin_port        = htons(ELOTTO_LINK_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),   /* also receives the broadcast */
    };
    if (bind(s_sock, (struct sockaddr *)&me, sizeof(me)) < 0) {
        ESP_LOGE(TAG, "bind(%d) failed", ELOTTO_LINK_CMD_PORT);
        close(s_sock);
        vTaskDelete(NULL);
        return;
    }
    TLOG("GCP-Slave ready  UDP port %d  source=%s\n",
         ELOTTO_LINK_CMD_PORT, (g_src == SRC_CAM) ? "camera" : "TRNG");

    char buf[ELOTTO_LINK_MAX];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        int n = recvfrom(s_sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &fl);
        if (n <= 0) continue;
        buf[n] = '\0';

        uint32_t seq;
        char    *cmd;
        if (!elotto_link_parse(buf, &seq, &cmd)) continue;   // foreign traffic

        // A repeat of the command we last answered: resend, do not re-execute.
        if (s_have_last && seq == s_last_seq) {
            link_reply(&from, seq, s_last_reply);
            continue;
        }

        if (cmd[0] == 'P') {
            link_reply(&from, seq, "OK");
            TLOG("discovery from %s -> OK\n", inet_ntoa(from.sin_addr));

        } else if (cmd[0] == 'K') {
            /* Camera calibration (docs/PLAN.md Task 1). The sweep itself lives
             * in the shared elotto_camera component, so this node and the master
             * run byte-identical logic and cannot disagree about what a
             * calibrated camera is — the same reason the extraction pipeline is
             * shared rather than duplicated.
             *
             * This node picks its OWN setting and will not match the master's.
             * That is correct: the cameras are physically different units, which
             * is precisely why one measured cleaner than the other at identical
             * settings. What must still be shared is the segment count per run,
             * and that travels on the wire. */
            int budget = atoi(cmd + 1);
            if (budget < 2000)   budget = 2000;
            if (budget > 120000) budget = 120000;
            g_abort = false;
            link_drain();       // a stale 'A' must not abort the sweep it precedes

            char r[96];
            if (!camera_is_ready() || !s_cal) {
                // Nothing to tune. Answered rather than ignored: silence here
                // would be counted as a missed reply and walk this node toward
                // being dropped for a reason that has nothing to do with it.
                snprintf(r, sizeof(r), "OK:0,0,0,0.500000,0.000,U");
                TLOG("cal: no camera (%s) -- nothing to calibrate\n",
                     s_cal ? "not streaming" : "no PSRAM for the table");
            } else {
                g_measuring = true;    // also refuses OTA while the sensor is
                                       // being reconfigured
                bool ok = camera_calibrate(budget, cal_abort_cb, s_cal);
                g_measuring = false;
                snprintf(r, sizeof(r), "OK:%lu,%lu,%d,%.6f,%.3f,%c",
                         (unsigned long)s_cal->exposure, (unsigned long)s_cal->gain,
                         s_cal->xor_fold ? 1 : 0, s_cal->bias, s_cal->mbit_per_sec,
                         ok ? 'G' : 'U');
                TLOG("cal done: exposure=%lu gain=%lu fold=%d %s (%lu ms, %d steps)\n",
                     (unsigned long)s_cal->exposure, (unsigned long)s_cal->gain,
                     s_cal->xor_fold ? 1 : 0,
                     ok ? "gated" : "NO gated setting -- kept previous",
                     (unsigned long)s_cal->elapsed_ms, s_cal->nsteps);
                log_camera_stats("after-calibration");
            }
            link_reply(&from, seq, r);

        } else if (cmd[0] == 'B') {
            int cnt = atoi(cmd + 1);
            if (cnt <= 0 || cnt > 5000) cnt = 100;
            const char *sarg = strchr(cmd, ',');
            g_abort = false;
            link_drain();   // a stale 'A' must not abort the session it precedes
            // Baseline marks the start of a session: re-arm the camera. Without
            // this, one transient stall latches this node to TRNG until power
            // cycle, and since the master now aborts any camera session whose
            // slave reports 'T', every later session would abort on arrival.
            if (camera_is_ready()) {
                g_src = SRC_CAM;
                g_src_fell_back  = false;
                g_fallback_logged = false;
            }
            // After the camera re-arm above, so the fallback default matches
            // the source this baseline will actually use.
            int nseg = seg_from_cmd(sarg ? sarg + 1 : NULL);
            g_measuring = true;
            double bsum = 0.0;
            int    done = 0;
            for (; done < cnt && !g_abort; done++)
                bsum += gcp_zscore_raw(nseg);
            g_measuring = false;
            g_baseline_mean = g_abort ? 0.0 : bsum / cnt;
            link_reply(&from, seq,"OK");
            TLOG("Baseline done: mean=%.4f (%d/%d runs, %d seg) source=%s\n",
                 g_baseline_mean, done, cnt, nseg,
                 (g_src == SRC_CAM) ? "camera" : "TRNG");
            log_camera_stats("after-baseline");

        } else if (cmd[0] == 'M') {
            g_abort = false;
            g_measuring = true;
            double z = gcp_zscore_raw(seg_from_cmd(cmd + 1)) - g_baseline_mean;
            g_measuring = false;
            // Reply carries the source this run actually used: 'C' camera,
            // 'T' TRNG. Appended after the float so atof() still parses it.
            char resp[40];
            snprintf(resp, sizeof(resp), "Z:%.6f,%c", z, (g_src == SRC_CAM) ? 'C' : 'T');
            link_reply(&from, seq,resp);
            if (g_src_fell_back && !g_fallback_logged) {
                g_fallback_logged = true;
                log_camera_stats("FALLBACK->TRNG");
            }

        } else if (cmd[0] == 'D') {
            // Camera diagnostics, so the master can surface slave-side health
            // instead of only seeing 'C'/'T' on each measurement.
            camera_stats_t cs;
            camera_get_stats(&cs);
            char r[128];
            snprintf(r, sizeof(r), "D:%d,%.6f,%.4f,%.3f,%lu,%lu,%c",
                     (int)cs.ready, cs.bias, cs.sigma, cs.mbit_per_sec,
                     (unsigned long)cs.stalls, (unsigned long)cs.stuck_frame_count,
                     (g_src == SRC_CAM) ? 'C' : 'T');
            link_reply(&from, seq, r);
            log_camera_stats("on-demand");

        } else if (cmd[0] == 'A') {
            g_abort = true;
            link_reply(&from, seq, "OK");

        } else {
            TLOG("Unknown command: '%c' (0x%02X)\n", cmd[0], (uint8_t)cmd[0]);
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* First: crash-loop protection, before anything that can itself crash. */
    elotto_ota_boot_check();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_eth_events = xEventGroupCreate();
    ethernet_init();

    // Camera entropy (this node's own OV5647 — never shared with the master).
    // Non-fatal: without it the slave still measures on its TRNG, which is the
    // pre-camera behaviour, and reports 'T' so the master can flag the mix.
    esp_err_t cam_ret = camera_init();
    if (cam_ret == ESP_OK) {
        s_cal = heap_caps_calloc(1, sizeof(camera_cal_t), MALLOC_CAP_SPIRAM);
        if (!s_cal) ESP_LOGE(TAG, "no PSRAM for the calibration table");
        // Wait briefly for the first frame pair so the ring has bits before the
        // master's first M command arrives.
        for (int i = 0; i < 100 && !camera_is_ready(); i++) vTaskDelay(pdMS_TO_TICKS(50));
    } else {
        ESP_LOGW(TAG, "camera_init: %s -- measuring on TRNG", esp_err_to_name(cam_ret));
    }
    g_src = camera_is_ready() ? SRC_CAM : SRC_TRNG;

    EventBits_t bits = xEventGroupWaitBits(s_eth_events, ETH_GOT_IP_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(30000));
    if (!(bits & ETH_GOT_IP_BIT)) {
        /* Deliberately do NOT mark valid, and do not start the command loop: an
         * image that cannot be reached is exactly the image that must roll back
         * to the recovery updater on the next boot. */
        ESP_LOGE(TAG, "no IP after 30 s — not validating this image");
        return;
    }

    httpd_handle_t srv = start_webserver();
    elotto_ota_register(srv, slave_busy);
    elotto_ota_mark_valid();

    if (g_src == SRC_CAM) {
        vTaskDelay(pdMS_TO_TICKS(3000));   // let stats accumulate before reporting
        log_camera_stats("idle-boot");     // light-leak baseline, no measurement load
    }

    // Own task rather than app_main: the consumer's priority is load-bearing
    // (must outrank ELOTTO_CAM_TASK_PRIO) and its stack has to hold the socket
    // path plus float formatting, neither of which app_main's defaults promise.
    xTaskCreate(link_task, "link", 6144, NULL, ELOTTO_CAM_TASK_PRIO + 1, NULL);
}
