/*
 * =============================================================================
 * tv_main_propa_win.c — Josh Throttle Valve Controller (Windows, Prop A)
 *
 * Windows port of tv_main_propa.c.
 * Replaces:
 *   SocketCAN  → PEAK PCAN-Basic SDK (PCANBasic.dll)
 *   POSIX clock/sleep → win_compat.h (QueryPerformanceCounter + WaitableTimer)
 *   POSIX sockets (GUI) → Winsock2
 *   POSIX signals → signal() (SIGINT only — SIGTERM not on Windows)
 *
 * Build with MinGW-w64:
 *   See Makefile.win
 *
 * Identical behaviour to tv_main_propa.c in all other respects.
 *
 * Identical to tv_main.c except valve commands use Prop A (DP=0, percent of
 * full open) instead of Prop A2 (DP=1, degrees).  The controller still works
 * in degrees internally — conversion happens only at the CAN send/receive
 * boundary:
 *   Command:  deg → pct = round(deg / 90.0 * 100.0), sent as data[0] 0-100
 *   Feedback: pct → deg = pct * 90.0 / 100.0, stored as actual_deg float
 *
 * Effective resolution: 1% × 90° = 0.9°/count — marginally coarser than
 * Prop A2's 1°/count.  Use tv_main_dither.c for finer effective resolution.
 *
 * Original file: tv_main.c
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
 *   INIT_IC → Operator-triggered: a one-shot Prop A2 command is sent driving
 *             both valves to THETA_O (LOX) and THETA_F (IPA) — the 500 lbf
 *             initial conditions that match the Simulink integrator ICs.
 *             Triggered from GUI after the operator has verified CAN comms
 *             (actual positions readable, FMI=0).  Returns to READY after
 *             the command is sent.  Valves slew at ~60 deg/s; monitor GUI.
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
 *   See Makefile.win  (MinGW-w64 required)
 *
 * DEPENDENCIES
 * ------------
 *   MinGW-w64:    https://www.mingw-w64.org  (or via MSYS2)
 *   libcurl:      https://curl.se/windows  (copy libcurl.dll to exe dir)
 *   PCAN-Basic:   https://www.peak-system.com/PCAN-Basic.239.0.html
 *                 (copy PCANBasic.dll to exe dir)
 *   PCAN driver:  Install PCAN_USB_Setup.exe from peak-system.com
 */

/* Windows port — include shims before anything else */
#include "win_compat.h"   /* POSIX clock/sleep/socket shims            */
#include "pcan_basic.h"   /* PEAK PCAN-Basic SDK for PCAN-USB Pro       */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>       /* included via win_compat.h — link with -lpthread */
#include <curl/curl.h>     /* libcurl for Windows  — link with -lcurl   */

/* ── can_frame shim (must precede kzvalve_can.h) ───────────────────────── *
 * kzvalve_can.h builds frames into a struct can_frame.  On Linux this is   *
 * the real SocketCAN type.  On Windows we define a compatible shim struct   *
 * with the same field names so kzvalve_can.h compiles unchanged.           *
 * CAN_EFF_FLAG / CAN_RTR_FLAG / CAN_ERR_FLAG are also defined here.        */
#define CAN_EFF_FLAG  0x80000000u
#define CAN_RTR_FLAG  0x40000000u
#define CAN_ERR_FLAG  0x20000000u

/* On Windows, struct can_frame is defined here as a shim for SocketCAN.
 * Name the struct tag 'can_frame' directly — no macro alias needed.
 * kzvalve_can.h uses 'struct can_frame' which resolves to this.         */
struct can_frame {
    uint32_t can_id;   /* EID with CAN_EFF_FLAG OR'd in */
    uint8_t  can_dlc;  /* Data Length Code (0..8)       */
    uint8_t  __pad[3];
    uint8_t  data[8];  /* Payload                       */
};

#include "tv_controller_2_1.h"
#include "kzvalve_can.h"

/* ── Prop A (percent mode) EID and frame builder ─────────────────────────── *
 * Prop A uses DP=0: EID base = (6<<26)|(0<<25)|(0<<24)|(0xEF<<16)           *
 *                            = 0x18EF0000                                    *
 * data[0] = position as percent of full open (0-100)                        *
 * data[1] = motor speed % (50-100)                                           *
 * data[2] = 0xFF reserved                                                    *
 * data[3] = MODE_ABSOLUTE (0x00)                                             *
 * data[4-7] = 0xFF reserved                                                  *
 * The periodic config (Mode 5) is still sent on Prop A2 per KZValve manual. */
#define EID_PROP_A_BASE     0x18EF0000u  /* Prop A (percent), DP=0 */

static inline struct can_frame kz_build_absolute_pct(
        uint8_t dest_sa, uint8_t src_sa,
        uint8_t pct, uint8_t speed_pct)
{
    struct can_frame f = {0};
    f.can_id  = (EID_PROP_A_BASE | ((uint32_t)dest_sa << 8) | src_sa) | CAN_EFF_FLAG;
    f.can_dlc = 8;
    f.data[0] = pct < 100 ? pct : 100;   /* percent 0-100 */
    f.data[1] = speed_pct < 50 ? 50 : (speed_pct > 100 ? 100 : speed_pct);
    f.data[2] = 0xFF;
    f.data[3] = MODE_ABSOLUTE;            /* 0x00 */
    f.data[4] = 0xFF;
    f.data[5] = 0xFF;
    f.data[6] = 0xFF;
    f.data[7] = 0xFF;
    return f;
}

/* Periodic config using Prop A EID (DP=0) — must match the command mode.
 * Same wire format as kz_build_periodic_cfg but with EID_PROP_A_BASE.     */
static inline struct can_frame kz_build_periodic_cfg_propa(
        uint8_t dest_sa, uint8_t src_sa, uint32_t period_ms)
{
    struct can_frame f = {0};
    f.can_id  = (EID_PROP_A_BASE | ((uint32_t)dest_sa << 8) | src_sa) | CAN_EFF_FLAG;
    f.can_dlc = 8;
    f.data[0] = (uint8_t)(period_ms & 0xFF);
    f.data[1] = (uint8_t)((period_ms >> 8)  & 0xFF);
    f.data[2] = (uint8_t)((period_ms >> 16) & 0xFF);
    f.data[3] = MODE_PERIODIC_CFG;   /* 0x05 */
    f.data[4] = 0xFF;
    f.data[5] = 0xFF;
    f.data[6] = 0xFF;
    f.data[7] = 0xFF;
    return f;
}

/* kz_build_request_propa — Request PGN frame per KZValve Prop A spec.
 *
 * Per KZValve documentation, the request frame must use DLC=8 with the
 * requested PGN in bytes 0-2 and trailing zeros (not the J1939 standard
 * DLC=3 format).  Requesting PGN 0x00EF00 asks for Prop A position feedback.
 *
 * Wire format confirmed by KZValve:
 *   EID:    0x18EABE01 (Pi→LOX) / 0x18EABF01 (Pi→IPA)
 *   data:   00 EF 00 00 00 00 00 00
 *           ↑──────┘ PGN 0x00EF00 (Prop A), bytes 4-8 = 0x00
 */
static inline struct can_frame kz_build_request_propa(
        uint8_t dest_sa, uint8_t src_sa)
{
    struct can_frame f = {0};
    /* EID_REQUEST_BASE = 0x18EA0000, DP=0, PF=0xEA */
    f.can_id  = (0x18EA0000u | ((uint32_t)dest_sa << 8) | src_sa) | CAN_EFF_FLAG;
    f.can_dlc = 8;                   /* KZValve requires DLC=8, not J1939 DLC=3 */
    f.data[0] = 0x00;                /* PGN 0x00EF00 LSB */
    f.data[1] = 0xEF;                /* PGN 0x00EF00 MID */
    f.data[2] = 0x00;                /* PGN 0x00EF00 MSB */
    f.data[3] = 0x00;                /* trailing zero */
    f.data[4] = 0x00;
    f.data[5] = 0x00;
    f.data[6] = 0x00;
    f.data[7] = 0x00;
    return f;
}

/* Suppress warn_unused_result on write() for GUI socket sends.
 * Network writes to a client socket are best-effort — if the client
 * disconnected we don't care about the partial write.               */
/* On Windows, send() must be used for Winsock sockets — write() does not work */
#define GUI_WRITE(fd, buf, n)  do { int _w = send((SOCKET)(fd),(const char*)(buf),(int)(n),0); (void)_w; } while(0)

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
/* PCAN-USB Pro channel — set this before building.
 * Common values:
 *   PCAN_USBBUS1 (0x51) — first CAN port   (default)
 *   PCAN_USBBUS2 (0x52) — second CAN port
 * Use PCAN-View (free from peak-system.com) to identify which port
 * is connected to the valve CAN bus.                                     */
#define CAN_CHANNEL         PCAN_USBBUS2

/* DAQstra sensor IDs — use exact sensor_id from /api/v1/sensors
 * Example IDs from b1_log_data_ads1256 board (ADS1256 ADC channels).
 * # must be URL-encoded as %23 in the REST path.
 * Update these to match your actual board/channel wiring.           */
#define SENSOR_ID_POM  "b1_log_data_ads1256%231"  /* LOX manifold (psi) */
#define SENSOR_ID_PFM  "b1_log_data_ads1256%232"  /* IPA manifold (psi) */
#define SENSOR_ID_PC   "b1_log_data_ads1256%233"  /* Chamber     (psi) */

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
#define SENSOR_TIMEOUT_MS   500         /* Max age of sensor reading before
                                         * FAULT is triggered during RUNNING.
                                         * 500ms = 33× the 15ms read cycle.  */

/* ══════════════════════════════════════════════════════════════════════════
 * DAQstra sensor cache — updated by sensor_thread, read by control loop
 *
 * The three HTTP sensor reads (POM, PFM, PC) each take 1-30ms on the Pi,
 * causing >97% overrun at 170 Hz if called synchronously in the control loop.
 * sensor_thread fetches all three at 170 Hz and writes to this cache.
 * The control loop reads the cache without blocking — worst case it uses a
 * value from the previous tick, which the 10-tap FIR smooths anyway.
 * ══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    double          pom_psi;
    double          pfm_psi;
    double          pc_psi;
    struct timespec last_update;   /* wall-clock time of last successful read */
    bool            ever_updated;  /* false until first real reading arrives  */
} SensorCache;

static SensorCache       g_sensors  = {370.7493, 370.8182, 322.8241, {0,0}, false};
static pthread_mutex_t   sensor_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ── Valve angle cache — written by can_process_rx(), read by daq_push_thread ──
 * Feeds actual valve positions (from real CAN feedback) into mock_daq so the
 * mock physics reflects what the real valves are doing.                     */
typedef struct { double lox_deg; double ipa_deg; } ValveAngles;
static ValveAngles     g_valve_angles = {0.0, 0.0};
static pthread_mutex_t valve_angle_mutex = PTHREAD_MUTEX_INITIALIZER;

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
    double      lox_mdot_kgs;   /* LOX mass flow estimate (kg/s) */
    double      ipa_mdot_kgs;   /* IPA mass flow estimate (kg/s) */

    /* Sensor readings (from DAQstra) */
    double      pom_psi;
    double      pfm_psi;
    double      pc_psi;

    /* Diagnostics */
    uint64_t    tick_count;
    uint64_t    can_send_count;
    uint64_t    overrun_count;
    double      loop_dt_ms;         /* actual last loop time */
    char        fault_reason[64];   /* human-readable fault cause for GUI     */

    /* FMI consecutive-frame counters — fault only after sustained bad FMI.
     * Resets to 0 when the FMI clears.  One counter per valve per FMI type. */
    uint8_t     lox_fmi_timeout_count;   /* FMI 7  consecutive frames, LOX */
    uint8_t     ipa_fmi_timeout_count;   /* FMI 7  consecutive frames, IPA */
    uint8_t     lox_fmi_voltage_count;   /* FMI 4  consecutive frames, LOX */
    uint8_t     ipa_fmi_voltage_count;   /* FMI 4  consecutive frames, IPA */
    uint8_t     lox_fmi_cal_count;       /* FMI 13 consecutive frames, LOX */
    uint8_t     ipa_fmi_cal_count;       /* FMI 13 consecutive frames, IPA */
} SharedState;

static SharedState   g_state;
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile bool   g_running   = true;

/* ══════════════════════════════════════════════════════════════════════════
 * CAN interface — PEAK PCAN-Basic API (Windows)
 *
 * Replaces the SocketCAN interface from the Linux version.
 *
 * Key differences from SocketCAN:
 *   open:  CAN_Initialize(channel, PCAN_BAUD_250K, 0, 0, 0)
 *   send:  CAN_Write(channel, &TPCANMsg)
 *   recv:  CAN_Read(channel, &TPCANMsg, &TPCANTimestamp)
 *          returns PCAN_ERROR_QRCVEMPTY when queue empty (like EAGAIN)
 *   close: CAN_Uninitialize(channel)
 *
 * The can_frame shim struct below lets kzvalve_can.h compile unchanged.
 * can_send_frame() translates it to a TPCANMsg before calling CAN_Write.
 * ══════════════════════════════════════════════════════════════════════════ */

/* can_frame shim and CAN_*_FLAG defines are hoisted to before kzvalve_can.h
 * (see above) so the struct is visible throughout the file.               */

static TPCANHandle g_can_channel = PCAN_NONEBUS;

static int can_open(TPCANHandle channel)
{
    /* For USB hardware: HwType=0, IOPort=0, Interrupt=0 */
    TPCANStatus st = CAN_Initialize(channel, PCAN_BAUD_250K, 0, 0, 0);
    if (st != PCAN_ERROR_OK) {
        char errtxt[256] = {0};
        CAN_GetErrorText(st, 0x09, errtxt);   /* 0x09 = English */
        fprintf(stderr, "[CAN] CAN_Initialize failed: 0x%08lX\n  %s\n", (unsigned long)st, errtxt);
        fprintf(stderr, "      Is PCAN-USB Pro connected and drivers installed?\n");
        fprintf(stderr, "      Install PCAN_USB_Setup.exe from peak-system.com\n");
        fprintf(stderr, "      Check CAN_CHANNEL define (currently 0x%02lX)\n", (unsigned long)channel);
        return -1;
    }
    g_can_channel = channel;
    printf("[CAN] Opened PCAN channel 0x%02lX at 250 kbps\n", (unsigned long)channel);
    return 0;
}

/* Translate a can_frame shim into a TPCANMsg and send via CAN_Write */
static int can_send_frame(const struct can_frame *f)
{
    TPCANMsg msg;
    msg.ID      = f->can_id & 0x1FFFFFFFu;   /* strip flag bits, keep 29-bit ID */
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;       /* always 29-bit extended (J1939)  */
    msg.LEN     = f->can_dlc;
    memcpy(msg.DATA, f->data, 8);
    TPCANStatus st = CAN_Write(g_can_channel, &msg);
    return (st == PCAN_ERROR_OK) ? 0 : -1;
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

    f = kz_build_periodic_cfg_propa(KZVALVE_SA_LOX, KZVALVE_SA_PI, CAN_PERIOD_MS);
    can_send_frame(&f);
    usleep(20000);

    f = kz_build_periodic_cfg_propa(KZVALVE_SA_IPA, KZVALVE_SA_PI, CAN_PERIOD_MS);
    can_send_frame(&f);
    usleep(20000);

    printf("[CAN] Periodic position broadcast configured at %d ms\n",
           CAN_PERIOD_MS);
}

/* Deadband state — file scope so can_drive_safe() can reset them.
 * Reset to 255 (sentinel) forces the next command to always send.     */
static uint8_t g_last_lox_pct = 255;
static uint8_t g_last_ipa_pct = 255;

/* ── Send absolute percent command to both valves (Prop A mode) ──────────── *
 * Converts the controller's degree output to percent of full open (0-100).  *
 * 1 count = 0.9 deg effective resolution (1% × 90°).                       *
 *                                                                            *
 * Deadband: only sends a new command if the rounded percent differs from     *
 * the last sent value.  Prevents the PI from toggling the valve between      *
 * adjacent counts (e.g. 44%↔45%) when the controller output sits near a     *
 * rounding boundary.  Without this, continuous ±1 count dither causes       *
 * FMI 7 (position timeout) because the valve never settles at its target.   *
 * Accuracy cost: up to 0.9° steady-state error; the PI integrator           *
 * compensates once it accumulates enough error to cross to the next count.  */
static void can_send_valve_commands(double lox_deg, double ipa_deg)
{
    /* Last-sent percent values — see file-scope g_last_lox_pct / g_last_ipa_pct */

    /* Clamp degrees to physical range, convert to percent */
    lox_deg = fmax(VALVE_MIN_DEG, fmin(VALVE_MAX_DEG, lox_deg));
    ipa_deg = fmax(VALVE_MIN_DEG, fmin(VALVE_MAX_DEG, ipa_deg));

    uint8_t lox_pct = (uint8_t)round(lox_deg / VALVE_MAX_DEG * 100.0);
    uint8_t ipa_pct = (uint8_t)round(ipa_deg / VALVE_MAX_DEG * 100.0);

    struct can_frame f;

    /* Only send if command changed — prevents adjacent-count dither */
    if (lox_pct != g_last_lox_pct) {
        f = kz_build_absolute_pct(KZVALVE_SA_LOX, KZVALVE_SA_PI,
                                   lox_pct, MOTOR_SPEED_PCT);
        can_send_frame(&f);
        g_last_lox_pct = lox_pct;
    }

    if (ipa_pct != g_last_ipa_pct) {
        f = kz_build_absolute_pct(KZVALVE_SA_IPA, KZVALVE_SA_PI,
                                   ipa_pct, MOTOR_SPEED_PCT);
        can_send_frame(&f);
        g_last_ipa_pct = ipa_pct;
    }
}

/* ── Drive both valves to safe state (0° = closed) ──────────────────────── */
static void can_drive_safe(void)
{
    /* Send 0-degree command to both valves.  The printf is rate-limited so
     * it fires only once per transition into safe state, not 170x/sec.     */
    static SystemState last_safe_state = (SystemState)-1;  /* sentinel: no state yet */
    /* Reset deadband so the next can_send_valve_commands call always sends. */
    g_last_lox_pct = 255;
    g_last_ipa_pct = 255;
    struct can_frame f;
    f = kz_build_absolute_pct(KZVALVE_SA_LOX, KZVALVE_SA_PI, 0, 100);
    can_send_frame(&f);
    f = kz_build_absolute_pct(KZVALVE_SA_IPA, KZVALVE_SA_PI, 0, 100);
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
    TPCANMsg     msg;
    TPCANTimestamp ts;
    struct can_frame f;
    /* CAN_Read returns PCAN_ERROR_QRCVEMPTY when queue is empty.
     * This is equivalent to read() returning -1/EAGAIN on Linux.      */
    while (CAN_Read(g_can_channel, &msg, &ts) == PCAN_ERROR_OK) {
        /* Skip non-extended and error/status frames */
        if (!(msg.MSGTYPE & PCAN_MESSAGE_EXTENDED)) continue;
        if (  msg.MSGTYPE & PCAN_MESSAGE_STATUS)    continue;
        /* Translate TPCANMsg back into can_frame shim so the rest of the
         * parsing logic (kz_src_addr, kz_pgn, kz_parse_position) is
         * identical to the Linux version.                              */
        f.can_id  = msg.ID | CAN_EFF_FLAG;
        f.can_dlc = msg.LEN;
        memcpy(f.data, msg.DATA, 8);
        if (f.can_id & CAN_ERR_FLAG) continue;  /* skip error frames */

        uint8_t  sa  = kz_src_addr(&f);
        uint32_t pgn = kz_pgn(&f);

        /* Prop A2 position feedback (PGN 0x01EF00, DP=1, PF=0xEF).
         * Only process frames from known valve addresses (LOX=0xBE, IPA=0xBF).
         * This prevents misinterpreting other CAN traffic as position feedback.
         * DLC must be 8 and the frame must not be an error frame.          */
        bool is_known_valve = (sa == KZVALVE_SA_LOX || sa == KZVALVE_SA_IPA);
        /* In Prop A mode the valve sends feedback as Prop A (DP=0, PGN 0x00EF00).
         * data[2] is actual position as percent (0-100); convert to degrees.
         * data[5] upper bits carry status flags — mask to lower 5 bits for
         * the standard J1939 FMI value (FMI is defined as 5-bit, 0-31).   */
        if ((pgn & 0x1FF00u) == 0x0EF00u && is_known_valve && f.can_dlc == 8) {
            uint8_t fmi_raw = 0;
            uint8_t pos_pct = kz_parse_position(&f, &fmi_raw);
            uint8_t fmi     = fmi_raw & 0x1Fu;   /* mask to 5-bit FMI field */
            uint8_t pos_deg = (uint8_t)round(pos_pct * 90.0 / 100.0);

            pthread_mutex_lock(&state_mutex);
            if (sa == KZVALVE_SA_LOX) {
                g_state.lox_actual_deg = pos_deg;
                g_state.lox_fmi        = fmi;
                g_state.lox_on_bus     = true;
            } else if (sa == KZVALVE_SA_IPA) {
                g_state.ipa_actual_deg = pos_deg;
                g_state.ipa_fmi        = fmi;
                g_state.ipa_on_bus     = true;
            }
            /* Note: valve angle cache for daq_push_thread is updated from
             * commanded angles in the main loop, not actual positions.
             * See step 5 of the main 170 Hz loop for the update.         */

            /* Fault detection with hysteresis — require N consecutive bad FMI
             * frames before transitioning to FAULT.  A single transient FMI
             * (e.g. during a setpoint step change) is ignored.
             *
             * Thresholds (at 10 Hz CAN feedback):
             *   FMI 7  (position timeout): 5 frames = 500ms sustained
             *   FMI 4  (under voltage):    3 frames = 300ms sustained
             *   FMI 13 (not calibrated):   2 frames = 200ms sustained
             *
             * Counters reset to 0 as soon as the FMI clears.               */
            #define FMI_THRESH_TIMEOUT  5
            #define FMI_THRESH_VOLTAGE  3
            #define FMI_THRESH_CAL      2

            uint8_t *cnt_timeout = (sa == KZVALVE_SA_LOX) ?
                &g_state.lox_fmi_timeout_count : &g_state.ipa_fmi_timeout_count;
            uint8_t *cnt_voltage = (sa == KZVALVE_SA_LOX) ?
                &g_state.lox_fmi_voltage_count : &g_state.ipa_fmi_voltage_count;
            uint8_t *cnt_cal     = (sa == KZVALVE_SA_LOX) ?
                &g_state.lox_fmi_cal_count     : &g_state.ipa_fmi_cal_count;

            /* Increment or reset each counter based on current FMI */
            if (fmi == FMI_POS_TIMEOUT)    (*cnt_timeout)++; else *cnt_timeout = 0;
            if (fmi == FMI_UNDER_VOLTAGE)  (*cnt_voltage)++; else *cnt_voltage = 0;
            if (fmi == FMI_NOT_CALIBRATED) (*cnt_cal)++;     else *cnt_cal     = 0;

            /* Trigger FAULT only when threshold reached and system is RUNNING */
            const char *fmi_name   = NULL;
            uint8_t     fmi_thresh = 0;
            if (*cnt_timeout >= FMI_THRESH_TIMEOUT) { fmi_name = "POSITION TIMEOUT"; fmi_thresh = FMI_POS_TIMEOUT; }
            else if (*cnt_voltage >= FMI_THRESH_VOLTAGE) { fmi_name = "UNDER VOLTAGE";    fmi_thresh = FMI_UNDER_VOLTAGE; }
            else if (*cnt_cal     >= FMI_THRESH_CAL)     { fmi_name = "NOT CALIBRATED";   fmi_thresh = FMI_NOT_CALIBRATED; }

            if (fmi_name && g_state.state == SYS_RUNNING) {
                g_state.state = SYS_FAULT;
                snprintf(g_state.fault_reason, sizeof(g_state.fault_reason),
                         "%s: valve SA=0x%02X FMI=%u", fmi_name, sa, fmi_thresh);
                fprintf(stderr, "[FAULT] %s\n", g_state.fault_reason);
                fprintf(stderr, "[FAULT] At fault: LOX cmd=%.2f act=%u  IPA cmd=%.2f act=%u\n",
                        g_state.lox_cmd_deg, g_state.lox_actual_deg,
                        g_state.ipa_cmd_deg, g_state.ipa_actual_deg);
                fprintf(stderr, "[FAULT] POM=%.2f PFM=%.2f PC=%.2f thrust_set=%.1f\n",
                        g_state.pom_psi, g_state.pfm_psi, g_state.pc_psi,
                        g_state.thrust_lbf_set);
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

#define DAQSTRA_MAX_URL 512

/* daqstra_init_handle — configure a persistent CURL handle for one sensor.
 * url_buf must be a caller-owned buffer of at least DAQSTRA_MAX_URL bytes
 * that outlives the handle — CURLOPT_URL does not copy the string.         */
static void daqstra_init_handle(CURL *c, const char *sensor_id, char *url_buf)
{
    snprintf(url_buf, DAQSTRA_MAX_URL, DAQSTRA_BASE "/api/v1/sensors/%s", sensor_id);
    curl_easy_setopt(c, CURLOPT_URL,           url_buf);
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, curl_write_cb);
    curl_easy_setopt(c, CURLOPT_TIMEOUT_MS,    30L);
    curl_easy_setopt(c, CURLOPT_NOSIGNAL,      1L);
    /* Keep-alive: reuse the TCP connection across 170 Hz requests           */
    curl_easy_setopt(c, CURLOPT_TCP_KEEPALIVE, 1L);
    curl_easy_setopt(c, CURLOPT_TCP_KEEPIDLE,  5L);
    curl_easy_setopt(c, CURLOPT_TCP_KEEPINTVL, 2L);
}

/* daqstra_get — fetch one sensor value using a pre-initialised handle.
 * The URL is already set; we only update WRITEDATA per call.              */
static double daqstra_get_by_id(CURL *curl, const char *sensor_id)
{
    (void)sensor_id;   /* URL already baked into handle by daqstra_init_handle */

    CurlBuf body = {NULL, 0};
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);

    CURLcode rc = curl_easy_perform(curl);
    double val = NAN;
    if (rc == CURLE_OK && body.buf) {
        char *p = strstr(body.buf, "\"latest_value\"");
        if (p) {
            p = strchr(p, ':');
            if (p) val = strtod(p + 1, NULL);
        }
    }
    free(body.buf);
    /* Do NOT call curl_easy_reset() — that drops the keep-alive connection */
    return val;
}

/* ══════════════════════════════════════════════════════════════════════════
 * DAQstra sensor background thread
 * Reads POM/PFM/PC via HTTP at 170 Hz and writes to g_sensors cache.
 * The control loop reads from g_sensors without blocking (<1 µs mutex read).
 * If an HTTP call takes longer than one 5.882ms tick the thread catches up
 * on the next clock_nanosleep — same hold-last-value behaviour as before.
 * ══════════════════════════════════════════════════════════════════════════ */
static void *sensor_thread(void *arg)
{
    (void)arg;

    /* Each handle owns its own persistent TCP connection to DAQstra.
     * URL buffers are thread-local and outlive the handles.           */
    CURL *c_pom = curl_easy_init();
    CURL *c_pfm = curl_easy_init();
    CURL *c_pc  = curl_easy_init();
    if (!c_pom || !c_pfm || !c_pc) {
        fprintf(stderr, "[DAQ] sensor_thread: curl_easy_init failed\n");
        return NULL;
    }
    char url_pom[DAQSTRA_MAX_URL];
    char url_pfm[DAQSTRA_MAX_URL];
    char url_pc [DAQSTRA_MAX_URL];
    daqstra_init_handle(c_pom, SENSOR_ID_POM, url_pom);
    daqstra_init_handle(c_pfm, SENSOR_ID_PFM, url_pfm);
    daqstra_init_handle(c_pc,  SENSOR_ID_PC,  url_pc);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long SENSOR_PERIOD_NS = (1000000000L / 170);  /* 5.882ms = 170 Hz */

    while (g_running) {
        double pom = daqstra_get_by_id(c_pom, SENSOR_ID_POM);
        double pfm = daqstra_get_by_id(c_pfm, SENSOR_ID_PFM);
        double pc  = daqstra_get_by_id(c_pc,  SENSOR_ID_PC);

        pthread_mutex_lock(&sensor_mutex);
        if (!isnan(pom)) g_sensors.pom_psi = pom;
        if (!isnan(pfm)) g_sensors.pfm_psi = pfm;
        if (!isnan(pc))  g_sensors.pc_psi  = pc;
        /* Stamp last_update whenever at least one sensor returned a valid value */
        if (!isnan(pom) || !isnan(pfm) || !isnan(pc)) {
            clock_gettime(CLOCK_MONOTONIC, &g_sensors.last_update);
            g_sensors.ever_updated = true;
        }
        pthread_mutex_unlock(&sensor_mutex);

        /* Sleep until next 170 Hz tick.  If HTTP took longer than one tick,
         * clock_nanosleep returns immediately and the thread self-corrects. */
        next.tv_nsec += SENSOR_PERIOD_NS;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    curl_easy_cleanup(c_pom);
    curl_easy_cleanup(c_pfm);
    curl_easy_cleanup(c_pc);
    return NULL;
}

/* Valve slew rate matching the KZValve EH22 at 100% speed (~60 deg/s per
 * kzvalve_can.h: "100 = fastest ~60 deg/s").  1.5s for full 90 deg stroke. */
#define DAQ_SLEW_DEG_PER_S  60.0

/* ══════════════════════════════════════════════════════════════════════════
 * DAQ push thread — feeds mock_daq with smoothly-slewing simulated angles
 *
 * WHY SMOOTH SLEW AND NOT ACTUAL CAN POSITIONS:
 * mock_daq computes pressure instantaneously from the angle it receives.
 * Posting actual CAN positions (10 Hz) causes pressure to step by ~34 psi
 * every 100ms — a large discrete jump that the PI integrators treat as a
 * sudden error, causing overshoot and oscillation.
 *
 * Instead this thread maintains its own simulated valve angles that slew
 * toward the commanded target at 60 deg/s (matching the real EH22), giving
 * mock_daq a smooth continuous ramp — exactly what the Simulink plant model
 * received during gain tuning.  Real CAN actual positions are used only for
 * GUI display (g_state.lox_actual_deg), not for the physics simulation.
 *
 * If mock_daq is not running, POST fails silently — no impact on control.
 * ══════════════════════════════════════════════════════════════════════════ */
static void *daq_push_thread(void *arg)
{
    (void)arg;
    CURL *curl = curl_easy_init();
    if (!curl) return NULL;

    curl_easy_setopt(curl, CURLOPT_URL,        DAQSTRA_BASE "/mock/valve_angles");
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 5L);     /* tight timeout — 170 Hz loop */
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL,   1L);
    curl_easy_setopt(curl, CURLOPT_POST,       1L);

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    /* Run at 170 Hz — same rate as Simulink plant, gives mock_daq smooth input */
    const long PUSH_PERIOD_NS = (1000000000L / 170);
    const double DT       = 1.0 / 170.0;
    const double MAX_STEP = DAQ_SLEW_DEG_PER_S * DT;   /* 0.353 deg/tick */

    /* Simulated angles start at ICs */
    double sim_lox = THETA_O;
    double sim_ipa = THETA_F;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    /* Adaptive rate: run at 170 Hz when mock_daq is responding, back off to
     * 1 Hz when it isn't (e.g. real DAQstra test — /mock/valve_angles → 404).
     * This prevents 170 failed HTTP calls/sec during real engine firings.    */
    int consecutive_failures = 0;
    const int BACKOFF_THRESHOLD = 10;   /* back off after 10 consecutive fails */

    while (g_running) {
        /* Read commanded angles written by main loop step 5b */
        pthread_mutex_lock(&valve_angle_mutex);
        double cmd_lox = g_valve_angles.lox_deg;
        double cmd_ipa = g_valve_angles.ipa_deg;
        pthread_mutex_unlock(&valve_angle_mutex);

        /* Slew simulated angles toward commanded at 60 deg/s */
        double dl = cmd_lox - sim_lox;
        double di = cmd_ipa - sim_ipa;
        if (fabs(dl) <= MAX_STEP) sim_lox = cmd_lox;
        else                      sim_lox += (dl > 0.0 ? MAX_STEP : -MAX_STEP);
        if (fabs(di) <= MAX_STEP) sim_ipa = cmd_ipa;
        else                      sim_ipa += (di > 0.0 ? MAX_STEP : -MAX_STEP);
        sim_lox = fmax(0.0, fmin(90.0, sim_lox));
        sim_ipa = fmax(0.0, fmin(90.0, sim_ipa));

        /* Post smooth angle to mock_daq */
        char body[128];
        snprintf(body, sizeof(body),
                 "{\"lox_deg\":%.4f,\"ipa_deg\":%.4f}", sim_lox, sim_ipa);
        curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, body);
        CURLcode rc = curl_easy_perform(curl);

        if (rc == CURLE_OK) {
            consecutive_failures = 0;
        } else {
            consecutive_failures++;
        }

        /* Sleep: 170 Hz if mock_daq is alive, 1 Hz if backing off */
        long period_ns;
        if (consecutive_failures < BACKOFF_THRESHOLD) {
            period_ns = PUSH_PERIOD_NS;            /* 5.88ms = 170 Hz */
        } else {
            period_ns = 1000000000L;               /* 1 s = 1 Hz backoff */
            /* Reset clock reference to avoid catching up after long sleep */
            clock_gettime(CLOCK_MONOTONIC, &next);
        }
        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Valve initialisation sequence
 * Wait for both valves on bus, then drive to Simulink ICs
 * ══════════════════════════════════════════════════════════════════════════ */
static bool valve_init_sequence(void)
{
    /* Request address claims from both valves.
     * The valve only broadcasts its address claim once at power-up, so if
     * tv_main starts after the valve is already running the claim is missed.
     * Sending a Request PGN for 0xEE00 forces the valve to re-broadcast.  */
    printf("[INIT] Requesting valve address claims...\n");
    struct can_frame req;
    req = kz_build_request(KZVALVE_SA_LOX, KZVALVE_SA_PI, PGN_ADDR_CLAIMED);
    can_send_frame(&req);
    usleep(10000);
    req = kz_build_request(KZVALVE_SA_IPA, KZVALVE_SA_PI, PGN_ADDR_CLAIMED);
    can_send_frame(&req);

    /* Wait up to 5 s for address claims */
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
        fprintf(stderr, "[INIT] WARNING: Not all valves found on bus — "
                        "continuing anyway.\n");
        /* Mark both valves on bus anyway — if commands are being accepted
         * the valve is present even if its address claim was missed.      */
        pthread_mutex_lock(&state_mutex);
        g_state.lox_on_bus = true;
        g_state.ipa_on_bus = true;
        pthread_mutex_unlock(&state_mutex);
    } else {
        printf("[INIT] Both valves on bus\n");
    }

    /* Configure periodic position broadcast at 100 ms regardless.
     * If the valve is responding to commands it will accept Mode 5.      */
    can_configure_periodic();
    usleep(100000);   /* 100ms — give valve time to process Mode 5         */

    /* Send explicit position requests immediately after Mode 5.
     * Per KZValve spec: Request PGN 0x00EF00 with DLC=8.
     * This confirms the valve is alive and triggers the first feedback.  */
    struct can_frame rq;
    rq = kz_build_request_propa(KZVALVE_SA_LOX, KZVALVE_SA_PI);
    can_send_frame(&rq);
    usleep(10000);
    rq = kz_build_request_propa(KZVALVE_SA_IPA, KZVALVE_SA_PI);
    can_send_frame(&rq);
    usleep(50000);
    can_process_rx();   /* read any immediate response */

    printf("[INIT] CAN ready. Verify valve positions in GUI, then press Init ICs.\n");
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * HTTP GUI server (minimal, single-threaded non-blocking accept)
 * ══════════════════════════════════════════════════════════════════════════ */
static SOCKET gui_sock = INVALID_SOCKET;

/* GUI served from gui.html on disk — see gui_handle_request() */

static void gui_init(void)
{
    gui_sock = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    /* On Windows, setsockopt requires (const char*) cast */
    (void)setsockopt(gui_sock, SOL_SOCKET, SO_REUSEADDR,
                     (const char*)&opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(GUI_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    (void)bind(gui_sock, (struct sockaddr *)&addr, sizeof(addr));
    (void)listen(gui_sock, 5);

    /* Non-blocking — use ioctlsocket() instead of fcntl() on Windows */
    socket_set_nonblocking(gui_sock);

    printf("[GUI] HTTP server on port %d\n", GUI_PORT);
}

/* Replace NaN/Inf with 0.0 so snprintf never emits "nan"/"inf" into JSON.
 * JSON spec does not allow these literals — the browser's JSON.parse() will
 * throw a SyntaxError and the GUI will show "No Connection".              */
static inline double jsafe(double v) { return (isfinite(v) ? v : 0.0); }

static void gui_build_status_json(char *buf, size_t sz)
{
    pthread_mutex_lock(&state_mutex);
    SharedState s = g_state;
    pthread_mutex_unlock(&state_mutex);

    /* Sanitize all doubles before formatting */
    s.lox_cmd_deg    = jsafe(s.lox_cmd_deg);
    s.ipa_cmd_deg    = jsafe(s.ipa_cmd_deg);
    s.mr_est         = jsafe(s.mr_est);
    s.thrust_est_lbf = jsafe(s.thrust_est_lbf);
    s.pom_psi        = jsafe(s.pom_psi);
    s.pfm_psi        = jsafe(s.pfm_psi);
    s.pc_psi         = jsafe(s.pc_psi);
    s.pom_set_psi    = jsafe(s.pom_set_psi);
    s.loop_dt_ms     = jsafe(s.loop_dt_ms);
    s.lox_mdot_kgs   = jsafe(s.lox_mdot_kgs);
    s.ipa_mdot_kgs   = jsafe(s.ipa_mdot_kgs);

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
        "\"lox_on_bus\":%s,\"ipa_on_bus\":%s,"
        "\"lox_mdot_kgs\":%.4f,\"ipa_mdot_kgs\":%.4f,"
        "\"fault_reason\":\"%s\""
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
        s.ipa_on_bus ? "true" : "false",
        s.lox_mdot_kgs,
        s.ipa_mdot_kgs,
        s.fault_reason
    );
}

static void gui_handle_request(int fd, const char *req, size_t req_len)
{
    (void)req_len;

    /* GET /api/status */
    if (strncmp(req, "GET /api/status", 15) == 0) {
        char json[1280];   /* enlarged for fault_reason field */
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
            /* Guard: block FIRE if either valve is more than 5 deg from IC.
             * If valves are not at ICs the integrators will immediately wind —
             * the operator must press Init ICs and wait for valves to settle. */
            double lox_err = fabs((double)g_state.lox_actual_deg - THETA_O);
            double ipa_err = fabs((double)g_state.ipa_actual_deg - THETA_F);
            if (lox_err > 5.0 || ipa_err > 5.0) {
                fprintf(stderr,
                    "[FIRE BLOCKED] Valves not at ICs: "
                    "LOX actual=%.1f° (target=%.1f°, err=%.1f°) "
                    "IPA actual=%.1f° (target=%.1f°, err=%.1f°)\n",
                    (double)g_state.lox_actual_deg, THETA_O, lox_err,
                    (double)g_state.ipa_actual_deg, THETA_F, ipa_err);
                pthread_mutex_unlock(&state_mutex);
                GUI_WRITE(fd,
                    "HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\n"
                    "BLOCKED: Valves not at ICs — press Init ICs and wait", 79);
                return;
            }
            /* Reinitialise Simulink model before every firing. */
            tv_controller_2_1_initialize();

            /* Pre-fill FIR filter buffers with current sensor readings.
             * initialize() zeros all buffers — without this, the first 10
             * ticks average zero + real readings, causing MR to spike and
             * the IPA integrator to wind to saturation before the FIR
             * converges (~58ms).  Pre-filling eliminates this transient.  */
            double pom_ic = g_state.pom_psi;
            double pfm_ic = g_state.pfm_psi;
            double pc_ic  = g_state.pc_psi;
            for (int i = 0; i < 9; i++) {
                tv_controller_2_1_DW.DiscreteFIRFilter1_states[i] = pom_ic;
                tv_controller_2_1_DW.DiscreteFIRFilter2_states[i] = pc_ic;
                tv_controller_2_1_DW.DiscreteFIRFilter_states[i]  = pfm_ic;
            }

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
        /* Reset Simulink integrators to ICs so the next FIRE starts clean.
         * Without this, wound-up integrators from this run would immediately
         * drive valves to saturation on the next firing.                   */
        tv_controller_2_1_initialize();
        GUI_WRITE(fd, "HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nOK", 41);
        return;
    }

    /* POST /api/cmd/reset — clear FAULT or ESTOP back to READY.
     * Resets FMI counters, fault_reason, and Simulink state.
     * Operator must re-verify system before arming again.              */
    if (strncmp(req, "POST /api/cmd/reset", 19) == 0) {
        pthread_mutex_lock(&state_mutex);
        if (g_state.state == SYS_FAULT || g_state.state == SYS_ESTOP) {
            g_state.state                = SYS_READY;
            g_state.control_enabled      = false;
            g_state.fault_reason[0]      = '\0';
            g_state.lox_fmi_timeout_count = 0;
            g_state.ipa_fmi_timeout_count = 0;
            g_state.lox_fmi_voltage_count = 0;
            g_state.ipa_fmi_voltage_count = 0;
            g_state.lox_fmi_cal_count     = 0;
            g_state.ipa_fmi_cal_count     = 0;
            tv_controller_2_1_initialize();
            printf("[CTRL] Fault cleared by operator — state → READY\n");
        }
        pthread_mutex_unlock(&state_mutex);
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

static void *gui_thread(void *arg)
{
    (void)arg;
    /* Set gui_sock to blocking so accept() waits efficiently.
     * Uses ioctlsocket() instead of fcntl() on Windows.              */
    socket_set_blocking(gui_sock);

    while (g_running) {
        struct sockaddr_in client;
        int clen = (int)sizeof(client);
        SOCKET cfd = accept(gui_sock, (struct sockaddr *)&client, &clen);
        if (cfd == INVALID_SOCKET) {
            int err = WSAGetLastError();
            /* WSAEINTR (10004) means the socket was closed — normal on shutdown */
            if (err == WSAEINTR || err == WSAENOTSOCK || err == WSAEINVAL) break;
            continue;
        }

        /* 200ms receive timeout — enough for any localhost HTTP request */
        struct timeval tv = {0, 200000};
        (void)setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO,
                         (const char*)&tv, sizeof(tv));

        char buf[2048] = {0};
        /* recv() instead of read() — read() does not work on Winsock sockets */
        int n = recv(cfd, buf, (int)(sizeof(buf) - 1), 0);
        if (n > 0) gui_handle_request((int)cfd, buf, (size_t)n);
        closesocket(cfd);
    }
    return NULL;
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
    /* SIGTERM is not available on Windows — SIGINT covers Ctrl+C      */

    /* ── Initialise Winsock2 FIRST — curl and GUI threads need it ──────── */
    winsock_init();

    /* ── Initialise libcurl ─────────────────────────────────────────────── */
    curl_global_init(CURL_GLOBAL_ALL);

    /* Spawn background sensor thread — reads DAQstra at 170 Hz into cache.
     * The thread creates its own curl handles internally.                  */
    pthread_t sensor_tid;
    if (pthread_create(&sensor_tid, NULL, sensor_thread, NULL) != 0) {
        fprintf(stderr, "[DAQ] Failed to create sensor thread\n"); return 1;
    }
    pthread_detach(sensor_tid);

    /* Spawn DAQ push thread — feeds actual valve positions to mock_daq */
    pthread_t daq_push_tid;
    if (pthread_create(&daq_push_tid, NULL, daq_push_thread, NULL) != 0) {
        fprintf(stderr, "[DAQ] Failed to create daq_push_thread\n"); return 1;
    }
    pthread_detach(daq_push_tid);

    /* Give the sensor thread time to populate the cache before the
     * control loop starts — one full read cycle takes ~3 × 5ms = 15ms.   */
    usleep(100000);   /* 100 ms — guarantees at least one full cache fill   */

    /* ── Open PCAN-USB Pro via PCAN-Basic SDK ───────────────────────────── */
    if (can_open(CAN_CHANNEL) < 0) {
        winsock_cleanup();
        return 1;   /* can_open() already printed diagnostics */
    }

    /* ── GUI server ─────────────────────────────────────────────────────── */
    gui_init();
    pthread_t gui_tid;
    if (pthread_create(&gui_tid, NULL, gui_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create GUI thread\n"); return 1;
    }
    pthread_detach(gui_tid);

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

        /* ── 1. Read PT sensors from cache (populated by sensor_thread) ────── *
         * sensor_thread fetches POM/PFM/PC at 170 Hz via HTTP and writes   *
         * to g_sensors.  Reading the cache here is non-blocking (<1 µs).   *
         * If an HTTP call took >5.88ms the previous value is held — same    *
         * behaviour as the original NaN fallback, without blocking the loop. */
        pthread_mutex_lock(&sensor_mutex);
        double pom = g_sensors.pom_psi;
        double pfm = g_sensors.pfm_psi;
        double pc  = g_sensors.pc_psi;
        pthread_mutex_unlock(&sensor_mutex);

        /* ── 2. Read current state under lock + sensor timeout check ──────── */
        pthread_mutex_lock(&state_mutex);
        g_state.pom_psi = pom;
        g_state.pfm_psi = pfm;
        g_state.pc_psi  = pc;
        double thrust_set = g_state.thrust_lbf_set;
        bool   ctrl_en    = g_state.control_enabled &&
                            (g_state.state == SYS_RUNNING);
        pthread_mutex_unlock(&state_mutex);

        /* Sensor dropout watchdog — only active when RUNNING.
         * Computes age of last sensor reading; if older than SENSOR_TIMEOUT_MS
         * transitions to FAULT.  Protects against silent DAQstra disconnection
         * during a firing where the controller would otherwise hold stale
         * pressure and silently wind integrators to wrong positions.          */
        pthread_mutex_lock(&sensor_mutex);
        bool    sensor_ever  = g_sensors.ever_updated;
        struct timespec s_ts = g_sensors.last_update;
        pthread_mutex_unlock(&sensor_mutex);

        if (ctrl_en && sensor_ever) {
            struct timespec now_ts;
            clock_gettime(CLOCK_MONOTONIC, &now_ts);
            double age_ms = (now_ts.tv_sec  - s_ts.tv_sec ) * 1e3 +
                            (now_ts.tv_nsec - s_ts.tv_nsec) * 1e-6;
            if (age_ms > (double)SENSOR_TIMEOUT_MS) {
                pthread_mutex_lock(&state_mutex);
                if (g_state.state == SYS_RUNNING) {
                    g_state.state = SYS_FAULT;
                    snprintf(g_state.fault_reason, sizeof(g_state.fault_reason),
                             "SENSOR DROPOUT: last reading %.0f ms ago", age_ms);
                    fprintf(stderr, "[FAULT] %s\n", g_state.fault_reason);
                }
                pthread_mutex_unlock(&state_mutex);
            }
        }

        /* ── 3. Feed Simulink inputs ─────────────────────────────────────── */
        tv_controller_2_1_U.thrust_lbf_set_inport = thrust_set;
        tv_controller_2_1_U.pom_psi_inport         = pom;
        tv_controller_2_1_U.pc_psi_inport          = pc;
        tv_controller_2_1_U.pfm_psi_inport         = pfm;

        /* ── 4. Step controller ──────────────────────────────────────────── *
         * Only run the Simulink model (and use its outputs) when RUNNING.   *
         * In any other state we hold commanded angles at the IC values so   *
         * the PI integrators cannot wind up due to zero sensor readings.    */
        double lox_cmd, ipa_cmd;
        if (ctrl_en) {
            tv_controller_2_1_step();
            lox_cmd = tv_controller_2_1_Y.lox_deg_outport;
            ipa_cmd = tv_controller_2_1_Y.ipa_deg_outport;
        } else {
            /* Freeze outputs at ICs — do not call step() to prevent wind-up */
            lox_cmd = THETA_O;
            ipa_cmd = THETA_F;
        }

        /* ── 5. Update shared state with controller outputs ─────────────── */
        pthread_mutex_lock(&state_mutex);
        g_state.lox_cmd_deg     = lox_cmd;
        g_state.ipa_cmd_deg     = ipa_cmd;
        if (ctrl_en) {
            /* Only update estimated quantities when controller is active */
            g_state.mr_est         = tv_controller_2_1_Y.mr_outport;
            g_state.thrust_est_lbf = tv_controller_2_1_Y.thrust_lbf_est_outport;
            g_state.pom_set_psi    = tv_controller_2_1_Y.pom_psi_set_outport;
            g_state.lox_mdot_kgs   = tv_controller_2_1_Y.lox_mdot_kgs_outport;
            g_state.ipa_mdot_kgs   = tv_controller_2_1_Y.ipa_mdot_kgs_outport;
        }
        g_state.tick_count++;
        pthread_mutex_unlock(&state_mutex);

        /* ── 5b. Update commanded-angle cache for daq_push_thread ─────────
         * The daq_push_thread slews its own simulated angles toward these
         * commanded values at 60 deg/s (matching the real valve) and posts
         * the smoothly-ramping simulated angles to mock_daq.               */
        pthread_mutex_lock(&valve_angle_mutex);
        g_valve_angles.lox_deg = lox_cmd;
        g_valve_angles.ipa_deg = ipa_cmd;
        pthread_mutex_unlock(&valve_angle_mutex);

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

            /* Poll for position every 5 CAN ticks (500ms) as a fallback.
             * If the valve missed the periodic config or periodic feedback
             * stops for any reason, this guarantees we still get position.
             * Request PGN 0x00EF00 per KZValve Prop A spec (DLC=8).      */
            static int poll_tick = 0;
            if (++poll_tick >= 5) {
                poll_tick = 0;
                struct can_frame rq;
                rq = kz_build_request_propa(KZVALVE_SA_LOX, KZVALVE_SA_PI);
                can_send_frame(&rq);
                rq = kz_build_request_propa(KZVALVE_SA_IPA, KZVALVE_SA_PI);
                can_send_frame(&rq);
            }
        }

        /* ── 7. GUI runs in gui_thread() pthread — nothing to do here ──────── */

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

    /* Closing gui_sock unblocks the gui_thread accept() so it exits */
    closesocket(gui_sock);

    tv_controller_2_1_terminate();
    curl_global_cleanup();
    if (g_can_channel != PCAN_NONEBUS)
        CAN_Uninitialize(g_can_channel);
    winsock_cleanup();

    printf("[CTRL] Clean exit\n");
    return 0;
}