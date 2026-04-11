#pragma once
/*
 * ============================================================================
 * kzvalve_can.h
 * Josh Throttle — KZValve EH2 J1939/ISO 11783 Protocol Layer
 * ============================================================================
 *
 * PURPOSE
 * -------
 * This header-only library provides all the CAN frame construction and parsing
 * functions needed to communicate with KZValve EH2 series smart valve actuators
 * over a J1939 CAN bus.  It is the ONLY file in the project that knows about
 * the KZValve wire protocol — tv_main.c calls these functions and never builds
 * raw CAN frames itself.
 *
 * PHYSICAL LAYER
 * --------------
 * - Protocol:  ISO 11783 / SAE J1939 (agricultural/industrial CAN standard)
 * - Baud rate: 250 kbps  (MUST match ip link set can0 type can bitrate 250000)
 * - Frame type: 29-bit extended ID (CAN 2.0B, CAN_EFF_FLAG must be set)
 * - Termination: 120 ohm resistor at EACH physical end of the CAN_H/CAN_L wire
 *   pair.  The EH2 does NOT have internal termination.
 *
 * DEGREE RESOLUTION (Prop A2)
 * ---------------------------
 * This library uses ISO 11783 Proprietary A2 (DP=1) throughout.  In Prop A2
 * mode each count in the commanded-position byte equals exactly 1 degree.
 * This is more useful than Prop A (percent-of-full-open) for a throttle valve
 * application where the flow vs. angle relationship is nonlinear and you need
 * to command specific angles.
 *
 *   Prop A  (DP=0, PGN 0x00EF00):  1 count = 1% of full open (0-100)
 *   Prop A2 (DP=1, PGN 0x01EF00):  1 count = 1 degree        (0-255)
 *   --> We always use Prop A2.
 *
 * J1939 ADDRESS ASSIGNMENTS
 * -------------------------
 *   LOX valve: 0xBE (190) — factory default, no change needed
 *   IPA valve: 0xBF (191) — must be commissioned (see Section 4 of docs)
 *   Pi ECU:    0x01 (1)   — claimed at startup by tv_main
 *
 * 29-BIT EXTENDED ID (EID) STRUCTURE
 * ------------------------------------
 * Bits [28:26]  Priority      3 bits  (0=highest, 7=lowest; we use 6)
 * Bit  [25]     EDP           1 bit   (always 0 for ISO 11783)
 * Bit  [24]     DP            1 bit   (0=Prop A percent, 1=Prop A2 degrees)
 * Bits [23:16]  PF            8 bits  (PDU Format — determines message type)
 * Bits [15:8]   PS            8 bits  (Destination Address if PF<240,
 *                                      Group Extension if PF>=240)
 * Bits  [7:0]   SA            8 bits  (Source Address — who sent this frame)
 *
 * Example: Prop A2 command from Pi (SA=0x01) to LOX valve (DA=0xBE):
 *   Priority=6, EDP=0, DP=1, PF=0xEF, PS=0xBE, SA=0x01
 *   EID = (6<<26)|(0<<25)|(1<<24)|(0xEF<<16)|(0xBE<<8)|0x01
 *       = 0x19EFBE01
 *
 * FAULT MODE INDICATOR (FMI) — WHAT IT IS
 * ----------------------------------------
 * The J1939 standard defines a set of numeric codes called Failure Mode
 * Identifiers (FMI) that describe HOW a component failed.  They are NOT
 * error codes in the software sense — they describe the physical failure mode.
 * The EH2 reports its FMI in:
 *   - Byte 6 (data[5]) of every periodic Prop A2 position feedback frame
 *   - Byte 5 (data[4], bits 1-5) of the DM1 diagnostic broadcast (1 Hz)
 *
 * FMI values used by KZValve:
 *   0  = No active fault                (normal)
 *   3  = Over voltage                   (supply above spec)
 *   4  = Under voltage                  (supply below spec — dangerous,
 *                                        position loop may be unreliable)
 *   7  = Mechanical timeout             (motor ran but valve didn't reach
 *                                        commanded angle in time — could mean
 *                                        propellant pressure load too high,
 *                                        mechanical binding, or gearbox issue)
 *   13 = Not calibrated                 (factory calibration not done —
 *                                        do not operate)
 *
 * FMI 7 is the most safety-critical for a propellant valve: it means the
 * actuator tried to move but physically could not reach the position.  This
 * should trigger an immediate abort.
 *
 * USAGE IN tv_main.c
 * ------------------
 * #include "kzvalve_can.h"
 *
 * // Command LOX valve to 45 degrees at full speed:
 * struct can_frame f = kz_build_absolute_deg(KZVALVE_SA_LOX, KZVALVE_SA_PI,
 *                                             45, 100);
 * write(can_sock, &f, sizeof(f));
 *
 * // Parse incoming feedback frame:
 * uint8_t fmi = 0;
 * uint8_t actual_deg = kz_parse_position(&f, &fmi);
 * if (fmi != 0) { // handle fault
 */

#include <stdint.h>
#include <stdbool.h>
/* On Linux use the real SocketCAN header.
 * On Windows the can_frame shim and CAN_*_FLAG defines are provided by
 * tv_main_propa_win.c before this header is included, so skip this.      */
#ifndef _WIN32
#include <linux/can.h>   /* struct can_frame, CAN_EFF_FLAG, CAN_RTR_FLAG */
#endif

/* ============================================================================
 * J1939 ADDRESS CONSTANTS
 * ============================================================================ */
#define KZVALVE_SA_LOX      0xBEu   /* LOX valve J1939 source address (190)   */
#define KZVALVE_SA_IPA      0xBFu   /* IPA valve J1939 source address (191)   */
#define KZVALVE_SA_PI       0x01u   /* Raspberry Pi ECU source address (1)    */
#define J1939_ADDR_GLOBAL   0xFFu   /* Global/broadcast address (255)         */

/* ============================================================================
 * PARAMETER GROUP NUMBERS (PGN)
 *
 * A PGN is the 18-bit identifier (EDP+DP+PF+PS) that defines what a message
 * contains.  Every device on the bus filters incoming frames by PGN and only
 * acts on the ones it supports.
 * ============================================================================ */

/* Prop A2 (DP=1): used for degree-mode absolute position commands AND for
 * the valve's periodic position feedback responses.  This is the primary PGN
 * for all normal valve control operations in this system.                     */
#define PGN_PROP_A2         0x01EF00u

/* Prop A (DP=0): percent-of-full-open mode.  NOT used in this system.
 * Included for reference only.                                                */
#define PGN_PROP_A          0x00EF00u

/* Address Claimed (PGN 0xEE00): broadcast by each device at startup to claim
 * its J1939 address.  tv_main listens for these to know when both valves are
 * on the bus and ready.                                                        */
#define PGN_ADDR_CLAIMED    0x00EE00u

/* Request PGN (0xEA00): used to ask a device to send a specific PGN.
 * tv_main can use this to poll position on demand rather than waiting for
 * the periodic broadcast.                                                      */
#define PGN_REQUEST         0x00EA00u

/* DM1 (0xFECA): Diagnostic Message 1 — broadcast by the valve once per second
 * regardless of fault status.  Contains the Suspect Parameter Number (SPN)
 * and Fault Mode Indicator (FMI).  In normal operation bytes 3-6 are zero.   */
#define PGN_DM1             0x00FECAu

/* DM14 (0xD900): Memory Access Request — used to read/write valve nonvolatile
 * memory.  tv_main uses this to configure the CAN-loss startup mode
 * (drive to 0° on CAN bus loss).                                               */
#define PGN_DM14            0x00D900u

/* DM16 (0xD700): Memory Access Data — carries the data payload for DM14
 * read/write operations.                                                        */
#define PGN_DM16            0x00D700u

/* ============================================================================
 * EID BASE VALUES
 *
 * These constants represent the fixed upper bits of each message's 29-bit EID.
 * The destination address (DA, bits [15:8]) and source address (SA, bits [7:0])
 * are filled in by make_eid() at runtime.
 *
 * Calculation for EID_PROP_A2_BASE:
 *   Priority = 6  → bits [28:26] = 110b  → (6 << 26) = 0x18000000
 *   EDP      = 0  → bit  [25]    = 0     → no contribution
 *   DP       = 1  → bit  [24]    = 1     → (1 << 24) = 0x01000000
 *   PF       = EF → bits [23:16]         → (0xEF << 16) = 0x00EF0000
 *   Sum = 0x18000000 + 0x01000000 + 0x00EF0000 = 0x19EF0000
 * ============================================================================ */
#define EID_PROP_A2_BASE    0x19EF0000u  /* Prop A2 (degrees), fill | (DA<<8) | SA */
#define EID_REQUEST_BASE    0x18EA0000u  /* Request PGN, DP=0, PF=0xEA             */
#define EID_DM14_BASE       0x18D90000u  /* Memory access request, DP=0, PF=0xD9   */
#define EID_DM16_BASE       0x18D70000u  /* Memory access data, DP=0, PF=0xD7      */
#define EID_ADDR_CLAIM_BASE 0x18EE0000u  /* Address claimed, PS must be 0xFF       */

/*
 * make_eid() — assemble the full 29-bit EID from a base constant + addresses.
 *
 * @base:    One of the EID_*_BASE constants above.
 * @dest_sa: Destination J1939 source address (goes into bits [15:8]).
 *           Use J1939_ADDR_GLOBAL (0xFF) for broadcast messages.
 * @src_sa:  Sender's J1939 source address (goes into bits [7:0]).
 *
 * The result must have CAN_EFF_FLAG OR'd in before passing to the kernel,
 * which all kz_build_* functions do automatically.
 */
static inline uint32_t make_eid(uint32_t base, uint8_t dest_sa, uint8_t src_sa)
{
    return base | ((uint32_t)dest_sa << 8) | src_sa;
}

/* ============================================================================
 * FAULT MODE INDICATOR (FMI) CONSTANTS
 *
 * These are the J1939-defined numeric codes that describe how an EH2 actuator
 * has failed.  The valve reports its FMI in Byte 6 (data[5]) of every
 * periodic Prop A2 feedback frame, and in the DM1 broadcast.
 *
 * ACTION REQUIRED for each code:
 *   FMI_NO_FAULT      → Normal, continue operation.
 *   FMI_OVER_VOLTAGE  → Log and warn, check power supply.
 *   FMI_UNDER_VOLTAGE → ABORT — valve position feedback may be unreliable.
 *   FMI_POS_TIMEOUT   → ABORT — valve physically could not reach target.
 *                        This is the most dangerous condition: the engine is
 *                        not receiving the correct propellant flow.
 *   FMI_NOT_CALIBRATED → Do not operate — return to KZValve for calibration.
 * ============================================================================ */
#define FMI_NO_FAULT        0u   /* No active fault — normal operation          */
#define FMI_OVER_VOLTAGE    3u   /* Supply voltage above rated maximum          */
#define FMI_UNDER_VOLTAGE   4u   /* Supply voltage below rated minimum          */
#define FMI_POS_TIMEOUT     7u   /* Valve timed out trying to reach position    */
#define FMI_NOT_CALIBRATED  13u  /* Factory calibration not performed           */

/* ============================================================================
 * PROP A / A2 MODE BYTE VALUES (Byte 4 / data[3] of command frame)
 *
 * The mode byte in a Prop A/A2 frame tells the valve which operation to perform.
 * tv_main only uses MODE_ABSOLUTE and MODE_PERIODIC_CFG.
 * ============================================================================ */
#define MODE_ABSOLUTE       0x00u  /* Move to specified degree angle            */
#define MODE_JOG_STOP       0x01u  /* Stop any jog motion                      */
#define MODE_JOG_OPEN       0x02u  /* Jog valve in opening direction            */
#define MODE_JOG_CLOSED     0x03u  /* Jog valve in closing direction            */
#define MODE_CHANGE_ADDR    0x04u  /* Change the valve's preferred J1939 address*/
#define MODE_PERIODIC_CFG   0x05u  /* Configure unsolicited position broadcasts */
#define MODE_LED_CTRL       0x06u  /* Control the status LED on the actuator    */

/* ============================================================================
 * NONVOLATILE MEMORY — CAN-LOSS STARTUP MODE
 *
 * The EH2 can be configured to automatically drive to a safe position when
 * the CAN bus goes silent (e.g. Pi crashes).  This is a HARDWARE safety net
 * that operates independently of software.
 *
 * Memory location 106 (MEM_ADDR_STARTUP) stores a 3-byte structure:
 *   Byte 2-3: Target position in degrees (0-359)
 *   Byte 4:   Mode (see STARTUP_MODE_* below)
 *
 * tv_main writes this configuration at startup via DM14/DM16 messages.
 * The user key (MEM_KEY_USER = 0x1234) must be included in DM14 to unlock
 * memory write access.
 * ============================================================================ */
#define MEM_ADDR_STARTUP      106u    /* Pointer value for DM14 Pointer LSB     */
#define MEM_KEY_USER          0x1234u /* Password required by DM14 to write     */
#define STARTUP_MODE_NONE     0u      /* Do nothing on startup or CAN loss       */
#define STARTUP_MODE_ON_START 1u      /* Drive to position on power-up only      */
#define STARTUP_MODE_CAN_LOSS 2u      /* Drive to position on CAN bus loss only  */
#define STARTUP_MODE_BOTH     3u      /* Drive to position on startup AND loss   */

/* ============================================================================
 * FRAME BUILDER: kz_build_absolute_deg
 *
 * Constructs a Prop A2 Absolute Mode command frame (KZValve manual Section 6.1).
 * This is the primary command sent by tv_main every 100ms (10 Hz) when the
 * system is in RUNNING state.
 *
 * Wire format (8 bytes):
 *   Byte 1 (data[0]): Commanded angle in degrees (0-255; valve range is 0-90)
 *   Byte 2 (data[1]): Motor speed as % of maximum (50-100; 100 = fastest ~60°/s)
 *   Byte 3 (data[2]): Reserved — always 0xFF
 *   Byte 4 (data[3]): Mode = 0x00 (Absolute)
 *   Bytes 5-8:        Reserved — always 0xFF
 *
 * @dest_sa:    Target valve address (KZVALVE_SA_LOX or KZVALVE_SA_IPA)
 * @src_sa:     Pi ECU address (KZVALVE_SA_PI = 0x01)
 * @deg:        Target valve angle in degrees (0=closed, 90=full open)
 * @speed_pct:  Motor speed 50-100; clamped to [50,100] if out of range
 *
 * Returns a fully assembled struct can_frame ready to write() to the socket.
 * ============================================================================ */
static inline struct can_frame kz_build_absolute_deg(
        uint8_t dest_sa, uint8_t src_sa,
        uint8_t deg, uint8_t speed_pct)
{
    struct can_frame f = {0};
    f.can_id  = make_eid(EID_PROP_A2_BASE, dest_sa, src_sa) | CAN_EFF_FLAG;
    f.can_dlc = 8;  /* Data Length Code — always 8 bytes for Prop A/A2 */
    f.data[0] = deg;
    /* Clamp speed to the acceptable range [50, 100] per KZValve manual */
    f.data[1] = speed_pct < 50 ? 50 : (speed_pct > 100 ? 100 : speed_pct);
    f.data[2] = 0xFF;              /* Reserved */
    f.data[3] = MODE_ABSOLUTE;     /* Mode byte = 0x00 */
    f.data[4] = 0xFF;              /* Reserved */
    f.data[5] = 0xFF;              /* Reserved */
    f.data[6] = 0xFF;              /* Reserved */
    f.data[7] = 0xFF;              /* Reserved */
    return f;
}

/* ============================================================================
 * FRAME BUILDER: kz_build_periodic_cfg
 *
 * Tells the valve to autonomously broadcast its position at a fixed interval
 * (KZValve manual Sections 5.5 / 6.2).  This is sent ONCE at startup.  After
 * this, the valve sends Prop A2 feedback frames every period_ms milliseconds
 * without any further polling.  tv_main configures 100ms (10 Hz).
 *
 * Wire format (8 bytes):
 *   Byte 1 (data[0]): Period LSB  (milliseconds, little-endian 24-bit value)
 *   Byte 2 (data[1]): Period Mid
 *   Byte 3 (data[2]): Period MSB
 *   Byte 4 (data[3]): Mode = 0x05 (Configure Periodic)
 *   Bytes 5-8:        Reserved — always 0xFF
 *
 * @dest_sa:    Target valve address
 * @src_sa:     Pi ECU address
 * @period_ms:  Broadcast period in milliseconds (10-60000; 100 = 10 Hz)
 *              Set all three bytes to 0xFF to DISABLE periodic transmission.
 * ============================================================================ */
static inline struct can_frame kz_build_periodic_cfg(
        uint8_t dest_sa, uint8_t src_sa, uint32_t period_ms)
{
    struct can_frame f = {0};
    f.can_id  = make_eid(EID_PROP_A2_BASE, dest_sa, src_sa) | CAN_EFF_FLAG;
    f.can_dlc = 8;
    /* 24-bit little-endian period value split across three bytes */
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

/* ============================================================================
 * FRAME BUILDER: kz_build_request
 *
 * Asks a valve (or all valves if dest_sa = J1939_ADDR_GLOBAL) to immediately
 * transmit a specific PGN.  Used to poll position on demand rather than waiting
 * for the next periodic broadcast (KZValve manual Section 8).
 *
 * Wire format (3 bytes — DLC=3 for Request PGN):
 *   Byte 1 (data[0]): Requested PGN LSB
 *   Byte 2 (data[1]): Requested PGN Mid
 *   Byte 3 (data[2]): Requested PGN MSB
 *
 * Example — request position from LOX valve:
 *   kz_build_request(KZVALVE_SA_LOX, KZVALVE_SA_PI, PGN_PROP_A2)
 * ============================================================================ */
static inline struct can_frame kz_build_request(
        uint8_t dest_sa, uint8_t src_sa, uint32_t pgn)
{
    struct can_frame f = {0};
    f.can_id  = make_eid(EID_REQUEST_BASE, dest_sa, src_sa) | CAN_EFF_FLAG;
    f.can_dlc = 3;   /* Request PGN uses only 3 bytes (the 3-byte PGN value) */
    f.data[0] = (uint8_t)(pgn & 0xFF);
    f.data[1] = (uint8_t)((pgn >> 8)  & 0xFF);
    f.data[2] = (uint8_t)((pgn >> 16) & 0xFF);
    return f;
}

/* ============================================================================
 * FRAME PARSER: kz_parse_position
 *
 * Extracts the actual valve position and fault status from an incoming Prop A2
 * feedback frame (KZValve manual Section 8.1 response format).
 *
 * The valve sends this frame periodically (after periodic_cfg is configured)
 * or in response to a Request PGN.  tv_main receives these in can_process_rx()
 * and feeds them into the shared state struct.
 *
 * Feedback frame layout (8 bytes):
 *   Byte 1 (data[0]): Commanded position (degrees) — what valve thinks Pi told it
 *   Byte 2 (data[1]): Commanded motor speed %
 *   Byte 3 (data[2]): ACTUAL position (degrees)   <-- THE PROCESS VARIABLE
 *   Byte 4 (data[3]): Mode = 0xFF
 *   Byte 5 (data[4]): Measured input voltage, rounded to nearest Volt (0-40V)
 *   Byte 6 (data[5]): Fault Mode Indicator (FMI) — 0 = no fault
 *   Bytes 7-8:        Reserved = 0xFF
 *
 * @f:          Pointer to received CAN frame.
 * @fault_fmi:  Output pointer for FMI code (may be NULL to ignore).
 *              Will be set to one of the FMI_* constants above.
 * Returns:     Actual valve position in degrees (Byte 3 = data[2]).
 * ============================================================================ */
static inline uint8_t kz_parse_position(const struct can_frame *f,
                                         uint8_t *fault_fmi)
{
    if (fault_fmi) *fault_fmi = f->data[5];  /* Byte 6 (0-indexed as [5]) = FMI */
    return f->data[2];                         /* Byte 3 (0-indexed as [2]) = actual position */
}

/* ============================================================================
 * HELPERS: kz_src_addr and kz_pgn
 *
 * These extract fields from received frame EIDs for routing/filtering.
 * tv_main uses them in can_process_rx() to decide which valve sent a frame
 * and what kind of message it is.
 * ============================================================================ */

/*
 * kz_src_addr — extract the sender's J1939 source address from a received frame.
 * The SA is always in bits [7:0] of the 29-bit EID.
 * Returns 0xBE for LOX valve, 0xBF for IPA valve.
 */
static inline uint8_t kz_src_addr(const struct can_frame *f)
{
    return (uint8_t)(f->can_id & 0xFFu);
}

/*
 * kz_pgn — extract the Parameter Group Number from a received frame EID.
 *
 * PGN extraction differs between PDU1 (PF < 240) and PDU2 (PF >= 240):
 *   PDU1: The PS byte is a Destination Address, NOT part of the PGN.
 *         PGN = EDP + DP + PF (PS excluded, zeroed).
 *   PDU2: The PS byte is a Group Extension, IS part of the PGN.
 *         PGN = EDP + DP + PF + PS (all 18 bits included).
 *
 * This distinction matters because Prop A/A2 (PF=0xEF < 240) is PDU1,
 * so the destination address in PS must be stripped before comparing to
 * PGN_PROP_A2.  DM1 (PF=0xFE >= 240) is PDU2, so PS is included.
 */
static inline uint32_t kz_pgn(const struct can_frame *f)
{
    uint32_t id = f->can_id & ~CAN_EFF_FLAG & ~CAN_RTR_FLAG;
    uint8_t  pf = (uint8_t)((id >> 16) & 0xFF);
    if (pf < 240) {
        /* PDU1: mask out PS (destination address), keep only EDP+DP+PF */
        return (id >> 8) & 0x3FF00u;
    } else {
        /* PDU2: PS is group extension — include it in the PGN */
        return (id >> 8) & 0x3FFFFu;
    }
}