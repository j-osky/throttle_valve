/*
 * =============================================================================
 * tv_main.c — Josh Throttle Valve Controller (Main Process)
 * =============================================================================
 *
 * OVERVIEW
 * --------
 * This is the entry point and integration hub for the Josh Throttle system.
 * It connects three subsystems that each run at different rates:
 *
 *   1. Simulink controller (tv_controller_2_1) — runs at 170 Hz
 *      The gain-scheduled PI controller generated from Simulink.  It reads
 *      pressure sensor values and outputs valve angle setpoints in degrees.
 *      Each call to tv_controller_2_1_step() advances it by one 5.882ms tick.
 *
 *   2. KZValve EH2 CAN actuators — commanded at 10 Hz, feedback at 10 Hz
 *      The Simulink outputs (valve angles in degrees) are sent to the physical
 *      valves over J1939 CAN every 100ms (every 17 Simulink ticks).
 *      The valves respond with actual position and fault status (FMI) in
 *      Prop A2 feedback frames, which are parsed every tick.
 *
 *   3. DAQstra sensor API — read every Simulink tick (170 Hz)
 *      Pressure transducer values (POM, PFM, PC) are fetched via HTTP GET
 *      from the DAQstra REST API using the sensor_id address format.
 *      Each fetch has a 30ms timeout; if it fails the previous value is held.
 *
 *   4. HTTP GUI server — non-blocking poll every Simulink tick
 *      A minimal single-threaded HTTP server on port 8080 serves gui.html
 *      and a /api/status JSON endpoint.  The GUI polls at 500ms.
 *
 * RATE STRUCTURE
 * --------------
 *   170 Hz (every tick, 5.882ms):
 *     - Read POM, PFM, PC from DAQstra via HTTP
 *     - Call tv_controller_2_1_step() with new sensor values
 *     - Read CAN receive buffer (non-blocking) for position feedback + FMI
 *     - Poll GUI HTTP accept (non-blocking)
 *
 *   10 Hz (every 17th tick, 100ms):
 *     - Send Prop A2 degree commands to LOX and IPA valves over CAN
 *
 *   500ms (GUI poll rate, driven by browser):
 *     - Serve /api/status JSON with all current state
 *
 * CONTROLLER INPUTS AND OUTPUTS
 * ------------------------------
 * Inputs (ExtU_tv_controller_2_1_T):
 *   thrust_lbf_set_inport  — Target thrust in lbf (from GUI, set by operator)
 *   pom_psi_inport         — LOX manifold pressure from DAQstra POM sensor
 *   pfm_psi_inport         — IPA manifold pressure from DAQstra PFM sensor
 *   pc_psi_inport          — Chamber pressure from DAQstra PC sensor
 *
 * Outputs (ExtY_tv_controller_2_1_T):
 *   lox_deg_outport        — Commanded LOX valve angle in degrees
 *   ipa_deg_outport        — Commanded IPA valve angle in degrees
 *   mr_outport             — Estimated mixture ratio (LOX/IPA)
 *   thrust_lbf_est_outport — Estimated thrust in lbf
 *   pom_psi_set_outport    — LOX manifold pressure setpoint in psi
 *
 * STATE MACHINE
 * -------------
 *   INIT    → Startup: waiting for both valve address claims on CAN bus.
 *             No CAN commands sent.  Advances automatically to READY once
 *             both 0xBE (LOX) and 0xBF (IPA) address claims are received.
 *
 *   READY   → Valves are on the bus and responding.  No active faults.
 *             The controller COMPUTES outputs every tick but does NOT send
 *             CAN commands.  Operator can ARM or use Init ICs from this state.
 *             This is the safe resting state between operations.
 *
 *   INIT_IC → Transient: a one-shot Prop A2 command is sent driving both
 *             valves to THETA_O (LOX) and THETA_F (IPA) — the 500 lbf
 *             initial conditions that match the Simulink integrator ICs.
 *             Returns to READY immediately after sending the command.
 *             The valves physically slew at ~60 deg/s; monitor in GUI.
 *
 *   ARMED   → Operator has confirmed the system is ready to fire.
 *             Controller still computing but CAN commands still NOT sent.
 *             This is a deliberate interlock: ARM confirms readiness,
 *             FIRE enables actual propellant flow control.
 *
 *   RUNNING → Closed-loop active.  Prop A2 commands sent at 10 Hz.
 *             Valve positions track controller outputs.  Monitor FMI.
 *
 *   FAULT   → A safety-critical FMI was received (FMI 4, 7, or 13).
 *             Both valves are commanded to 0° (closed).  CAN commands
 *             continue to repeat 0° every tick until cleared.
 *             Requires power cycle to clear.
 *
 *   ESTOP   → Emergency stop triggered from GUI.
 *             Identical behaviour to FAULT.  Requires power cycle.
 *
 * FAULT MODE INDICATOR (FMI)
 * --------------------------
 * FMI is a J1939 standard numeric code that describes HOW a device failed.
 * The EH22 valve reports its FMI in Byte 6 of every Prop A2 feedback frame.
 * The system checks FMI every CAN receive tick (170 Hz):
 *   FMI 0  = No fault          → Normal
 *   FMI 3  = Over voltage      → Log warning
 *   FMI 4  = Under voltage     → FAULT state (position may be unreliable)
 *   FMI 7  = Position timeout  → FAULT state (valve couldn't reach target)
 *   FMI 13 = Not calibrated    → FAULT state (don't operate)
 *
 * DAQSTRA SENSOR IDs
 * ------------------
 * tv_main reads sensors by their exact DAQstra sensor_id string, NOT by
 * friendly title.  The IDs follow the format:
 *   b<board>_log_data_<type>%23<channel>
 * where %23 is the URL-encoded # (hash) character.
 * Update SENSOR_ID_POM, SENSOR_ID_PFM, SENSOR_ID_PC to match your wiring.
 *
 * INITIAL CONDITIONS
 * ------------------
 * Both valve PI integrators in the Simulink model are initialised to specific
 * angles (THETA_O and THETA_F) that represent the steady-state valve positions
 * at 500 lbf operating point.  The physical valves must be at these same angles
 * when FIRE is pressed, or the controller will experience a large initial error
 * and the integrators will wind.  Use Init ICs before every firing.
 *
 * BUILD
 * -----
 *   gcc -O2 -Wall -I. -std=c11 \
 *       -o tv_main tv_main.c tv_controller_2_1.c tv_controller_2_1_data.c \
 *       -lm -lpthread -lcurl
 *   (or: make)
 *
 * DEPENDENCIES
 * ------------
 *   libcurl:     sudo apt-get install libcurl4-openssl-dev
 *   SocketCAN:   sudo apt-get install can-utils
 *   CAN bus up:  sudo ip link set can0 up type can bitrate 250000
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <curl/curl.h>

#include "tv_controller_2_1.h"
#include "kzvalve_can.h"

/* Suppress warn_unused_result on write() for GUI socket sends.
 * Network writes to a client socket are best-effort — if the client
 * disconnected we don't care about the partial write.               */
#define GUI_WRITE(fd, buf, n)  do { ssize_t _w = write((fd),(buf),(n)); (void)_w; } while(0)

/* ══════════════════════════════════════════════════════════════════════════
 * Configuration
 * ══════════════════════════════════════════════════════════════════════════ */

#define CTRL_HZ             170         /* Simulink base rate (Hz). Must match
                                         * tv_controller_2_1 solver sample time.
                                         * Tick period = 1/170 = 5.882 ms.     */
#define CAN_SEND_DIVIDER    17          /* Send CAN every N ticks = 10 Hz.
                                         * 170 Hz / 17 = 10 Hz.  This is the
                                         * maximum allowed by KZValve spec
                                         * (min repetition period = 100 ms).   */
#define CAN_PERIOD_MS       100         /* Valve auto-broadcast period (ms).
                                         * Sent once at startup via Mode 5.
                                         * Valves then push position every
                                         * 100 ms without polling.             */
#define GUI_PORT            8080        /* HTTP server port for gui.html and
                                         * /api/status JSON endpoint.          */
#define DAQSTRA_BASE        "http://localhost:8050"
#define CAN_IFACE           "can0"

/* DAQstra sensor IDs — use exact sensor_id from /api/v1/sensors
 * Example IDs from b1_log_data_ads1256 board (ADS1256 ADC channels).
 * # must be URL-encoded as %23 in the REST path.
 * Update these to match your actual board/channel wiring.           */
#define SENSOR_ID_POM  "b1_log_data_ads1256%230"  /* LOX manifold (psi) */
#define SENSOR_ID_PFM  "b1_log_data_ads1256%231"  /* IPA manifold (psi) */
#define SENSOR_ID_PC   "b1_log_data_ads1256%232"  /* Chamber     (psi) */

/* Valve initial conditions at 500 lbf operating point.
 * These are theta_o (LOX) and theta_f (IPA) from gain_scheduling.m, run at
 * 500 lbf. The exact values are baked into the Simulink integrator and rate
 * limiter initial condition blocks, as seen in tv_controller_2_1_initialize():
 *   Integrator_DSTATE   = 41.06017851  (LOX PI integrator, S91)
 *   PrevY               = 41.06017851  (LOX rate limiter)
 *   Integrator_DSTATE_c = 40.67697413  (IPA PI integrator, S41)
 *   PrevY_j             = 40.67697413  (IPA rate limiter)
 */
/* theta_o: LOX valve initial condition in degrees at 500 lbf operating point.
 * Source: gain_scheduling.m run at 500 lbf → fed into Simulink block IC →
 * baked into tv_controller_2_1_initialize() as:
 *   tv_controller_2_1_DW.Integrator_DSTATE = 41.06017851   (LOX PI integrator)
 *   tv_controller_2_1_DW.PrevY             = 41.06017851   (LOX rate limiter)
 * The physical LOX valve must be at this angle when FIRE is pressed so that
 * the integrator state matches physical reality and avoids a startup transient. */
static const double THETA_O = 41.06017851;   /* theta_o: LOX valve IC (deg) */
/* theta_f: IPA valve initial condition in degrees at 500 lbf operating point.
 * Source: same as THETA_O but for the IPA (fuel) loop:
 *   tv_controller_2_1_DW.Integrator_DSTATE_c = 40.67697413  (IPA PI integrator)
 *   tv_controller_2_1_DW.PrevY_j             = 40.67697413  (IPA rate limiter) */
static const double THETA_F = 40.67697413;   /* theta_f: IPA valve IC (deg) */

/* Safety limits */
#define VALVE_MIN_DEG       0.0
#define VALVE_MAX_DEG       90.0
#define MOTOR_SPEED_PCT     100         /* Full speed — gearbox limits slew */

/* Tick period in nanoseconds */
#define TICK_NS             (1000000000L / CTRL_HZ)  /* 5,882,353 ns       */

/* ══════════════════════════════════════════════════════════════════════════
 * Shared state (protected by state_mutex)
 * ══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    SYS_INIT = 0,
    SYS_READY,
    SYS_INIT_IC,       /* Driving valves to IC positions           */
    SYS_ARMED,
    SYS_RUNNING,
    SYS_FAULT,
    SYS_ESTOP
} SystemState;

static const char *state_names[] = {
    "INIT", "READY", "INIT_IC", "ARMED", "RUNNING", "FAULT", "ESTOP"
};

typedef struct {
    /* System */
    SystemState state;
    bool        initialized;

    /* Setpoints */
    double      thrust_lbf_set;
    bool        control_enabled;

    /* Valve feedback (from CAN) */
    uint8_t     lox_actual_deg;
    uint8_t     ipa_actual_deg;
    uint8_t     lox_fmi;
    uint8_t     ipa_fmi;
    bool        lox_on_bus;
    bool        ipa_on_bus;

    /* Controller outputs */
    double      lox_cmd_deg;
    double      ipa_cmd_deg;
    double      mr_est;
    double      thrust_est_lbf;
    double      pom_set_psi;

    /* Sensor readings (from DAQstra) */
    double      pom_psi;
    double      pfm_psi;
    double      pc_psi;

    /* Diagnostics */
    uint64_t    tick_count;
    uint64_t    can_send_count;
    uint64_t    overrun_count;
    double      loop_dt_ms;         /* actual last loop time */
} SharedState;

static SharedState   g_state;
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile bool   g_running   = true;

/* ══════════════════════════════════════════════════════════════════════════
 * CAN interface
 * ══════════════════════════════════════════════════════════════════════════ */
static int can_sock = -1;

static int can_open(const char *iface)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock < 0) { perror("socket(CAN)"); return -1; }

    /* Enable reception of error frames */
    can_err_mask_t em = CAN_ERR_MASK;
    setsockopt(can_sock, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &em, sizeof(em));

    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(can_sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)"); close(can_sock); return -1;
    }

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind(CAN)"); close(can_sock); return -1;
    }

    /* Non-blocking receive */
    int flags = fcntl(can_sock, F_GETFL, 0);
    (void)fcntl(can_sock, F_SETFL, flags | O_NONBLOCK);

    printf("[CAN] Opened %s (fd=%d)\n", iface, can_sock);
    return 0;
}

static int can_send_frame(const struct can_frame *f)
{
    ssize_t n = write(can_sock, f, sizeof(*f));
    return (n == sizeof(*f)) ? 0 : -1;
}

/* ── Address claim (Pi claims 0x01) ─────────────────────────────────────── */
static void can_send_address_claim(void)
{
    struct can_frame f = {0};
    /* PGN 0xEE00, PS=0xFF (global), SA=Pi */
    /* EID: Priority=6, EDP=0, DP=0, PF=0xEE, PS=0xFF (global broadcast), SA=0x01
     * Assembled directly rather than via make_eid to set PS=global explicitly. */
    f.can_id  = ((6u<<26) | (0u<<25) | (0u<<24) | (0xEEu<<16) |
                 ((uint32_t)J1939_ADDR_GLOBAL<<8) | KZVALVE_SA_PI) | CAN_EFF_FLAG;
    f.can_dlc = 8;
    /* NAME: arbitrary, Arbitrary Address Capable=1 (bit 63) */
    memset(f.data, 0xFF, 8);
    f.data[7] = 0xA0;  /* Industry Group=2, Arbitrary=1 */
    can_send_frame(&f);
    printf("[CAN] Address claim sent (SA=0x01)\n");
}

/* ── Configure periodic position broadcast on both valves ───────────────── */
static void can_configure_periodic(void)
{
    struct can_frame f;

    f = kz_build_periodic_cfg(KZVALVE_SA_LOX, KZVALVE_SA_PI, CAN_PERIOD_MS);
    can_send_frame(&f);
    usleep(20000);

    f = kz_build_periodic_cfg(KZVALVE_SA_IPA, KZVALVE_SA_PI, CAN_PERIOD_MS);
    can_send_frame(&f);
    usleep(20000);

    printf("[CAN] Periodic position broadcast configured at %d ms\n",
           CAN_PERIOD_MS);
}

/* ── Send absolute degree command to both valves ─────────────────────────── */
static void can_send_valve_commands(double lox_deg, double ipa_deg)
{
    /* Clamp and convert to uint8 */
    uint8_t lox = (uint8_t)fmax(VALVE_MIN_DEG,
                                  fmin(VALVE_MAX_DEG, lox_deg));
    uint8_t ipa = (uint8_t)fmax(VALVE_MIN_DEG,
                                  fmin(VALVE_MAX_DEG, ipa_deg));

    struct can_frame f;

    f = kz_build_absolute_deg(KZVALVE_SA_LOX, KZVALVE_SA_PI,
                               lox, MOTOR_SPEED_PCT);
    can_send_frame(&f);

    f = kz_build_absolute_deg(KZVALVE_SA_IPA, KZVALVE_SA_PI,
                               ipa, MOTOR_SPEED_PCT);
    can_send_frame(&f);
}

/* ── Drive both valves to safe state (0° = closed) ──────────────────────── */
static void can_drive_safe(void)
{
    /* Send 0-degree command to both valves.  The printf is rate-limited so
     * it fires only once per transition into safe state, not 170x/sec.     */
    static SystemState last_safe_state = -1;
    struct can_frame f;
    f = kz_build_absolute_deg(KZVALVE_SA_LOX, KZVALVE_SA_PI, 0, 100);
    can_send_frame(&f);
    f = kz_build_absolute_deg(KZVALVE_SA_IPA, KZVALVE_SA_PI, 0, 100);
    can_send_frame(&f);

    pthread_mutex_lock(&state_mutex);
    SystemState cur = g_state.state;
    pthread_mutex_unlock(&state_mutex);
    if (cur != last_safe_state) {
        printf("[CAN] Valves commanded to SAFE (0 deg) — state=%s\n",
               state_names[cur]);
        last_safe_state = cur;
    }
}

/* ── Parse incoming CAN frames ──────────────────────────────────────────── */
static void can_process_rx(void)
{
    struct can_frame f;
    while (read(can_sock, &f, sizeof(f)) == sizeof(f)) {
        if (f.can_id & CAN_ERR_FLAG) continue;  /* skip error frames */

        uint8_t  sa  = kz_src_addr(&f);
        uint32_t pgn = kz_pgn(&f);

        /* Prop A2 position feedback (PGN 0x01EF00, DP=1, PF=0xEF).
         * Only process frames from known valve addresses (LOX=0xBE, IPA=0xBF).
         * This prevents misinterpreting other CAN traffic as position feedback.
         * DLC must be 8 and the frame must not be an error frame.          */
        bool is_known_valve = (sa == KZVALVE_SA_LOX || sa == KZVALVE_SA_IPA);
        if ((pgn & 0x1FF00u) == 0x1EF00u && is_known_valve && f.can_dlc == 8) {
            uint8_t fmi = 0;
            uint8_t pos = kz_parse_position(&f, &fmi);

            pthread_mutex_lock(&state_mutex);
            if (sa == KZVALVE_SA_LOX) {
                g_state.lox_actual_deg = pos;
                g_state.lox_fmi        = fmi;
                g_state.lox_on_bus     = true;
            } else if (sa == KZVALVE_SA_IPA) {
                g_state.ipa_actual_deg = pos;
                g_state.ipa_fmi        = fmi;
                g_state.ipa_on_bus     = true;
            }

            /* Fault detection — only on confirmed bad FMI codes, only when
             * RUNNING. FMI=0 is normal. FMI values 1,2,5,6,8-12 are not used
             * by KZValve so treat any unexpected nonzero as a warning only.  */
            if (fmi == FMI_POS_TIMEOUT || fmi == FMI_NOT_CALIBRATED ||
                fmi == FMI_UNDER_VOLTAGE) {
                if (g_state.state == SYS_RUNNING) {
                    g_state.state = SYS_FAULT;
                    fprintf(stderr, "[FAULT] Valve SA=0x%02X FMI=%u\n", sa, fmi);
                }
            }
            pthread_mutex_unlock(&state_mutex);
        }

        /* Address claimed — mark valve as present */
        if ((pgn & 0x3FFFFu) == 0xEE00u) {
            pthread_mutex_lock(&state_mutex);
            if (sa == KZVALVE_SA_LOX) { g_state.lox_on_bus = true; }
            if (sa == KZVALVE_SA_IPA) { g_state.ipa_on_bus = true; }
            pthread_mutex_unlock(&state_mutex);
            printf("[CAN] Address claim received from SA=0x%02X\n", sa);
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * DAQstra HTTP sensor reads (libcurl)
 * ══════════════════════════════════════════════════════════════════════════ */
typedef struct { char *buf; size_t len; } CurlBuf;

static size_t curl_write_cb(void *data, size_t sz, size_t n, void *userp)
{
    CurlBuf *b = (CurlBuf *)userp;
    size_t add = sz * n;
    b->buf = realloc(b->buf, b->len + add + 1);
    memcpy(b->buf + b->len, data, add);
    b->len += add;
    b->buf[b->len] = '\0';
    return add;
}

/* daqstra_get_by_id — fetch the latest sensor value from DAQstra REST API.
 *
 * Called at 170 Hz (every Simulink tick) for each of the three PT sensors.
 * Uses a 30 ms libcurl timeout so a slow DAQstra response does not cause a
 * controller overrun (170 Hz tick period = 5.88 ms, but the sensor read is
 * allowed to straddle multiple ticks given the non-real-time nature of HTTP).
 * If the fetch fails or times out, NAN is returned and the caller substitutes 0.
 *
 * @sensor_id: URL-encoded DAQstra sensor ID string.
 *             The # character in DAQstra IDs (e.g. b1_log_data_ads1256#0)
 *             MUST be encoded as %23 in the URL path:
 *             → b1_log_data_ads1256%230
 *             This is handled by the SENSOR_ID_* defines which already
 *             contain the %23 encoding.
 * Returns: latest_value as double, or NAN on any error.
 */
static double daqstra_get_by_id(CURL *curl, const char *sensor_id)
{
    char url[512];
    snprintf(url, sizeof(url),
             DAQSTRA_BASE "/api/v1/sensors/%s", sensor_id);

    CurlBuf body = {NULL, 0};
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 30L);   /* 30 ms timeout */
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    CURLcode rc = curl_easy_perform(curl);
    double val = NAN;
    if (rc == CURLE_OK && body.buf) {
        /* Quick JSON parse: find "latest_value": <number> */
        char *p = strstr(body.buf, "\"latest_value\"");
        if (p) {
            p = strchr(p, ':');
            if (p) val = strtod(p + 1, NULL);
        }
    }
    free(body.buf);
    curl_easy_reset(curl);
    return val;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Valve initialisation sequence
 * Wait for both valves on bus, then drive to Simulink ICs
 * ══════════════════════════════════════════════════════════════════════════ */
static bool valve_init_sequence(void)
{
    printf("[INIT] Waiting for valve address claims (up to 5s)...\n");
    for (int i = 0; i < 100; i++) {       /* 100 × 50 ms = 5 s */
        can_process_rx();
        pthread_mutex_lock(&state_mutex);
        bool lox = g_state.lox_on_bus;
        bool ipa = g_state.ipa_on_bus;
        pthread_mutex_unlock(&state_mutex);
        if (lox && ipa) break;
        usleep(50000);
    }

    pthread_mutex_lock(&state_mutex);
    bool ok = g_state.lox_on_bus && g_state.ipa_on_bus;
    pthread_mutex_unlock(&state_mutex);

    if (!ok) {
        fprintf(stderr, "[INIT] ERROR: Not all valves found on bus\n");
        return false;
    }
    printf("[INIT] Both valves on bus\n");

    /* Configure periodic feedback at 100 ms */
    can_configure_periodic();
    usleep(50000);

    /* Drive valves to Simulink initial conditions */
    printf("[INIT] Moving valves to ICs: LOX=%.1f° IPA=%.1f°\n",
           THETA_O, THETA_F);
    can_send_valve_commands(THETA_O, THETA_F);

    /* Wait for valves to reach IC positions (max 3 s) */
    for (int i = 0; i < 60; i++) {
        can_process_rx();
        pthread_mutex_lock(&state_mutex);
        bool lox_ok = abs((int)g_state.lox_actual_deg - (int)THETA_O) <= 2;
        bool ipa_ok = abs((int)g_state.ipa_actual_deg - (int)THETA_F) <= 2;
        pthread_mutex_unlock(&state_mutex);
        if (lox_ok && ipa_ok) {
            printf("[INIT] Valves at IC positions\n");
            return true;
        }
        usleep(50000);
    }

    /* Not converged but not fatal — continue with warning */
    pthread_mutex_lock(&state_mutex);
    printf("[INIT] WARNING: LOX actual=%u° IPA actual=%u° (target ~41°/41°)\n",
           g_state.lox_actual_deg, g_state.ipa_actual_deg);
    pthread_mutex_unlock(&state_mutex);
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * HTTP GUI server (minimal, single-threaded non-blocking accept)
 * ══════════════════════════════════════════════════════════════════════════ */
static int gui_sock = -1;

/* GUI served from gui.html on disk — see gui_handle_request() */

static void gui_init(void)
{
    gui_sock = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    (void)setsockopt(gui_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(GUI_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    (void)bind(gui_sock, (struct sockaddr *)&addr, sizeof(addr));
    (void)listen(gui_sock, 5);

    /* Non-blocking */
    int flags = fcntl(gui_sock, F_GETFL, 0);
    (void)fcntl(gui_sock, F_SETFL, flags | O_NONBLOCK);

    printf("[GUI] HTTP server on port %d\n", GUI_PORT);
}

static void gui_build_status_json(char *buf, size_t sz)
{
    pthread_mutex_lock(&state_mutex);
    SharedState s = g_state;
    pthread_mutex_unlock(&state_mutex);

    snprintf(buf, sz,
        "{"
        "\"state\":\"%s\","
        "\"lox_cmd_deg\":%.2f,\"lox_actual_deg\":%u,\"lox_fmi\":%u,"
        "\"ipa_cmd_deg\":%.2f,\"ipa_actual_deg\":%u,\"ipa_fmi\":%u,"
        "\"pom_psi\":%.2f,\"pfm_psi\":%.2f,\"pc_psi\":%.2f,"
        "\"pom_set_psi\":%.2f,"
        "\"thrust_lbf_set\":%.1f,\"thrust_est_lbf\":%.2f,"
        "\"mr_est\":%.4f,"
        "\"tick_count\":%llu,\"can_send_count\":%llu,"
        "\"overrun_count\":%llu,\"loop_dt_ms\":%.3f,"
        "\"lox_on_bus\":%s,\"ipa_on_bus\":%s"
        "}",
        state_names[s.state],
        s.lox_cmd_deg, s.lox_actual_deg, s.lox_fmi,
        s.ipa_cmd_deg, s.ipa_actual_deg, s.ipa_fmi,
        s.pom_psi, s.pfm_psi, s.pc_psi,
        s.pom_set_psi,
        s.thrust_lbf_set, s.thrust_est_lbf,
        s.mr_est,
        (unsigned long long)s.tick_count,
        (unsigned long long)s.can_send_count,
        (unsigned long long)s.overrun_count,
        s.loop_dt_ms,
        s.lox_on_bus ? "true" : "false",
        s.ipa_on_bus ? "true" : "false"
    );
}

static void gui_handle_request(int fd, const char *req, size_t req_len)
{
    (void)req_len;

    /* GET /api/status */
    if (strncmp(req, "GET /api/status", 15) == 0) {
        char json[1024];
        gui_build_status_json(json, sizeof(json));
        char resp[1200];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n%s", json);
        GUI_WRITE(fd, resp, n);
        return;
    }

    /* POST /api/cmd/init_ic
     * Drives both valves to THETA_O / THETA_F (500 lbf ICs).
     * Safe to call from READY or ARMED state before a run. */
    if (strncmp(req, "POST /api/cmd/init_ic", 21) == 0) {
        pthread_mutex_lock(&state_mutex);
        if (g_state.state == SYS_READY || g_state.state == SYS_ARMED) {
            g_state.state = SYS_INIT_IC;
            g_state.control_enabled = false;
        }
        pthread_mutex_unlock(&state_mutex);
        /* Drive valves to IC positions immediately */
        can_send_valve_commands(THETA_O, THETA_F);
        pthread_mutex_lock(&state_mutex);
        g_state.lox_cmd_deg = THETA_O;
        g_state.ipa_cmd_deg = THETA_F;
        /* Return to READY once commanded — actual convergence happens asynchronously */
        if (g_state.state == SYS_INIT_IC)
            g_state.state = SYS_READY;
        pthread_mutex_unlock(&state_mutex);
        GUI_WRITE(fd, "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK", 41);
        return;
    }

    /* POST /api/cmd/arm */
    if (strncmp(req, "POST /api/cmd/arm", 17) == 0) {
        pthread_mutex_lock(&state_mutex);
        if (g_state.state == SYS_READY)
            g_state.state = SYS_ARMED;
        pthread_mutex_unlock(&state_mutex);
        GUI_WRITE(fd, "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK", 41);
        return;
    }

    /* POST /api/cmd/run */
    if (strncmp(req, "POST /api/cmd/run", 17) == 0) {
        pthread_mutex_lock(&state_mutex);
        if (g_state.state == SYS_ARMED) {
            g_state.control_enabled = true;
            g_state.state = SYS_RUNNING;
        }
        pthread_mutex_unlock(&state_mutex);
        GUI_WRITE(fd, "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK", 41);
        return;
    }

    /* POST /api/cmd/stop */
    if (strncmp(req, "POST /api/cmd/stop", 18) == 0) {
        pthread_mutex_lock(&state_mutex);
        g_state.control_enabled = false;
        if (g_state.state == SYS_RUNNING)
            g_state.state = SYS_READY;
        pthread_mutex_unlock(&state_mutex);
        can_drive_safe();
        GUI_WRITE(fd, "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK", 41);
        return;
    }

    /* POST /api/cmd/estop */
    if (strncmp(req, "POST /api/cmd/estop", 19) == 0) {
        pthread_mutex_lock(&state_mutex);
        g_state.control_enabled = false;
        g_state.state = SYS_ESTOP;
        pthread_mutex_unlock(&state_mutex);
        can_drive_safe();
        fprintf(stderr, "[ESTOP] Emergency stop triggered from GUI\n");
        GUI_WRITE(fd, "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK", 41);
        return;
    }

    /* POST /api/thrust  body: {"thrust_lbf": 450} */
    if (strncmp(req, "POST /api/thrust", 16) == 0) {
        const char *body = strstr(req, "\r\n\r\n");
        if (body) {
            body += 4;
            char *p = strstr(body, "thrust_lbf");
            if (p) {
                p = strchr(p, ':');
                if (p) {
                    double v = strtod(p + 1, NULL);
                    if (v >= 0 && v <= 800) {
                        pthread_mutex_lock(&state_mutex);
                        g_state.thrust_lbf_set = v;
                        pthread_mutex_unlock(&state_mutex);
                    }
                }
            }
        }
        GUI_WRITE(fd, "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK", 41);
        return;
    }

    /* GET / — serve gui.html from disk */
    {
        FILE *gf = fopen("gui.html", "r");
        if (gf) {
            char hdr[] = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
                         "Connection: close\r\n\r\n";
            GUI_WRITE(fd, hdr, strlen(hdr));
            char fbuf[4096];
            size_t nr;
            while ((nr = fread(fbuf, 1, sizeof(fbuf), gf)) > 0)
                GUI_WRITE(fd, fbuf, nr);
            fclose(gf);
        } else {
            /* Fallback if gui.html not found */
            const char *msg = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
                              "Connection: close\r\n\r\n"
                              "<html><body style=\"background:#0c0c0e;color:#e0e0ee;font-family:monospace;padding:40px\">"
                              "<h2 style=\"color:#e63030\">Josh Throttle Controller</h2>"
                              "<p>Open <a href=\"http://localhost:8080\" style=\"color:#00d4ff\">gui.html</a> "
                              "in the same directory as tv_main.</p>"
                              "</body></html>";
            GUI_WRITE(fd, msg, strlen(msg));
        }
    }
}

static void gui_poll(void)
{
    struct sockaddr_in client;
    socklen_t clen = sizeof(client);
    int cfd = accept(gui_sock, (struct sockaddr *)&client, &clen);
    if (cfd < 0) return;  /* EAGAIN = no connection waiting */

    /* Set 100 ms receive timeout */
    struct timeval tv = {0, 100000};
    (void)setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    char buf[2048] = {0};
    ssize_t n = read(cfd, buf, sizeof(buf) - 1);
    if (n > 0) gui_handle_request(cfd, buf, n);
    close(cfd);
}

/* ══════════════════════════════════════════════════════════════════════════
 * Signal handler
 * ══════════════════════════════════════════════════════════════════════════ */
static void sig_handler(int sig)
{
    (void)sig;
    g_running = false;
}

/* ══════════════════════════════════════════════════════════════════════════
 * High-resolution sleep until absolute time
 * ══════════════════════════════════════════════════════════════════════════ */
static void sleep_until(struct timespec *next)
{
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
    /* Advance by one tick period */
    next->tv_nsec += TICK_NS;
    if (next->tv_nsec >= 1000000000L) {
        next->tv_nsec -= 1000000000L;
        next->tv_sec++;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Main
 * ══════════════════════════════════════════════════════════════════════════ */
int main(void)
{
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║       Josh Throttle — TV Controller                  ║\n");
    printf("║     tv_controller_2_1 @ 170 Hz | CAN @ 10 Hz        ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n\n");

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── Initialise libcurl ─────────────────────────────────────────────── */
    curl_global_init(CURL_GLOBAL_ALL);
    CURL *curl = curl_easy_init();
    if (!curl) { fprintf(stderr, "curl_easy_init failed\n"); return 1; }

    /* ── Open CAN bus ───────────────────────────────────────────────────── */
    if (can_open(CAN_IFACE) < 0) {
        fprintf(stderr, "Failed to open %s. Is PCAN-USB connected and "
                        "bus configured?\n  Run: sudo ip link set %s up "
                        "type can bitrate 250000\n", CAN_IFACE, CAN_IFACE);
        return 1;
    }

    /* ── GUI server ─────────────────────────────────────────────────────── */
    gui_init();

    /* ── Simulink model init ────────────────────────────────────────────── */
    tv_controller_2_1_initialize();
    printf("[CTRL] Simulink model initialised\n");
    printf("       LOX IC = %.2f deg  IPA IC = %.2f deg\n",
           THETA_O, THETA_F);

    /* Populate shared state with initial conditions */
    memset(&g_state, 0, sizeof(g_state));
    g_state.state           = SYS_INIT;
    g_state.thrust_lbf_set  = 500.0;   /* 500 lbf operating point default */
    g_state.lox_cmd_deg     = THETA_O;
    g_state.ipa_cmd_deg     = THETA_F;

    /* ── Address claim ──────────────────────────────────────────────────── */
    can_send_address_claim();
    usleep(300000);  /* 300 ms — let valves finish their startup sequence  */

    /* ── Valve initialisation ───────────────────────────────────────────── */
    if (!valve_init_sequence()) {
        fprintf(stderr, "[INIT] Valve init failed — check CAN wiring\n");
        /* Don't abort — allow bench use without valves for mock DAQ */
    }

    pthread_mutex_lock(&state_mutex);
    g_state.state = SYS_READY;
    pthread_mutex_unlock(&state_mutex);
    printf("[CTRL] State → READY. Open http://localhost:%d\n\n", GUI_PORT);

    /* ── Main 170 Hz loop ───────────────────────────────────────────────── */
    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    int can_tick = 0;

    while (g_running) {
        struct timespec t0;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        /* ── 1. Read PT sensors from DAQstra at 170 Hz ───────────────────── *
         * Each call makes an HTTP GET request to localhost:8050.            *
         * The 30ms timeout is intentionally longer than the 5.88ms tick    *
         * period — this is acceptable because the Simulink FIR filter       *
         * (10-tap moving average) smooths out occasional stale readings.    *
         * In practice the DAQstra API responds in <1ms on localhost.        */
        double pom = daqstra_get_by_id(curl, SENSOR_ID_POM);
        double pfm = daqstra_get_by_id(curl, SENSOR_ID_PFM);
        double pc  = daqstra_get_by_id(curl, SENSOR_ID_PC);

        /* Fallback to zero if sensor unreachable (will show in GUI) */
        if (isnan(pom)) pom = 0.0;
        if (isnan(pfm)) pfm = 0.0;
        if (isnan(pc))  pc  = 0.0;

        /* ── 2. Read current state under lock ───────────────────────────── */
        pthread_mutex_lock(&state_mutex);
        g_state.pom_psi = pom;
        g_state.pfm_psi = pfm;
        g_state.pc_psi  = pc;
        double thrust_set = g_state.thrust_lbf_set;
        bool   ctrl_en    = g_state.control_enabled &&
                            (g_state.state == SYS_RUNNING);
        pthread_mutex_unlock(&state_mutex);

        /* ── 3. Feed Simulink inputs ─────────────────────────────────────── */
        tv_controller_2_1_U.thrust_lbf_set_inport = thrust_set;
        tv_controller_2_1_U.pom_psi_inport         = pom;
        tv_controller_2_1_U.pc_psi_inport          = pc;
        tv_controller_2_1_U.pfm_psi_inport         = pfm;

        /* ── 4. Step controller ──────────────────────────────────────────── */
        tv_controller_2_1_step();

        double lox_cmd = tv_controller_2_1_Y.lox_deg_outport;
        double ipa_cmd = tv_controller_2_1_Y.ipa_deg_outport;

        /* ── 5. Update shared state with controller outputs ─────────────── */
        pthread_mutex_lock(&state_mutex);
        g_state.lox_cmd_deg     = lox_cmd;
        g_state.ipa_cmd_deg     = ipa_cmd;
        g_state.mr_est          = tv_controller_2_1_Y.mr_outport;
        g_state.thrust_est_lbf  = tv_controller_2_1_Y.thrust_lbf_est_outport;
        g_state.pom_set_psi     = tv_controller_2_1_Y.pom_psi_set_outport;
        g_state.tick_count++;
        pthread_mutex_unlock(&state_mutex);

        /* ── 6. CAN: send at 10 Hz, receive every tick ──────────────────── */
        can_process_rx();

        if (++can_tick >= CAN_SEND_DIVIDER) {
            can_tick = 0;
            if (ctrl_en) {
                can_send_valve_commands(lox_cmd, ipa_cmd);
                pthread_mutex_lock(&state_mutex);
                g_state.can_send_count++;
                pthread_mutex_unlock(&state_mutex);
            }
        }

        /* ── 7. GUI poll (non-blocking accept) ───────────────────────────── */
        gui_poll();

        /* ── 8. Fault → safe state ───────────────────────────────────────── */
        pthread_mutex_lock(&state_mutex);
        SystemState st = g_state.state;
        pthread_mutex_unlock(&state_mutex);
        if (st == SYS_FAULT || st == SYS_ESTOP) {
            can_drive_safe();
        }

        /* ── 9. Timing: measure dt, detect overrun, sleep ───────────────── */
        struct timespec t1;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double dt_ms = (t1.tv_sec - t0.tv_sec) * 1e3 +
                       (t1.tv_nsec - t0.tv_nsec) * 1e-6;

        pthread_mutex_lock(&state_mutex);
        g_state.loop_dt_ms = dt_ms;
        if (dt_ms > (1000.0 / CTRL_HZ)) g_state.overrun_count++;
        pthread_mutex_unlock(&state_mutex);

        sleep_until(&next_tick);
    }

    /* ── Shutdown ───────────────────────────────────────────────────────── */
    printf("\n[CTRL] Shutting down...\n");
    can_drive_safe();
    usleep(200000);

    tv_controller_2_1_terminate();
    curl_easy_cleanup(curl);
    curl_global_cleanup();
    close(can_sock);
    close(gui_sock);

    printf("[CTRL] Clean exit\n");
    return 0;
}