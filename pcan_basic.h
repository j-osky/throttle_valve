/*
 * pcan_basic.h — PEAK PCAN-Basic SDK types and declarations (Windows)
 * =============================================================================
 * This header declares the PCAN-Basic API used by tv_main_propa_win.c.
 * The actual implementation is in PCANBasic.dll, which ships with the free
 * PEAK PCAN-Basic SDK.
 *
 * SDK DOWNLOAD
 * ------------
 * https://www.peak-system.com/PCAN-Basic.239.0.html
 *
 * SETUP STEPS
 * -----------
 * 1. Download and run the PCAN-Basic SDK installer from peak-system.com.
 * 2. Install the PCAN-USB Pro driver (PCAN_USB_Setup.exe) from the same site.
 * 3. Copy PCANBasic.dll to the same directory as tv_main_propa_win.exe.
 *    DLL location after install: C:\Program Files\PEAK-System\PCAN-Basic API\Win32\ (or x64\)
 * 4. Link with -lPCANBasic when building.
 *    Place PCANBasic.lib in the build directory (from the SDK \Lib\MSVC\ or \Lib\MinGW\ folder).
 *
 * This file provides the minimal subset of PCANBasic.h needed by
 * tv_main_propa_win.c.  It can be replaced with the official PCANBasic.h
 * from the SDK (same declarations, more complete).
 *
 * PCAN-USB PRO CHANNEL ASSIGNMENTS
 * ---------------------------------
 * The PCAN-USB Pro has two independent CAN channels:
 *   PCAN_USBBUS1 (0x51) — CAN port 1  (default — set CAN_CHANNEL to this)
 *   PCAN_USBBUS2 (0x52) — CAN port 2
 *
 * Use PCAN-View (free, from peak-system.com) to identify which physical
 * port is connected to the valve CAN bus before building.
 * =============================================================================
 */

#pragma once
#include <windows.h>

/* ── Primitive types ─────────────────────────────────────────────────────── */
typedef DWORD   TPCANHandle;        /* CAN channel handle                   */
typedef DWORD   TPCANStatus;        /* Return/status code                   */
typedef BYTE    TPCANMessageType;   /* Frame type flags                     */
typedef DWORD   TPCANBaudrate;      /* Baud rate constant                   */
typedef DWORD   TPCANType;          /* Hardware type                        */
typedef DWORD   TPCANParameter;     /* Configuration parameter ID           */
typedef DWORD   TPCANMode;          /* Filter mode                          */

/* ── Channel handles ─────────────────────────────────────────────────────── */
#define PCAN_NONEBUS    ((TPCANHandle)0x00u)

/* USB channels */
#define PCAN_USBBUS1    ((TPCANHandle)0x51u)   /* PCAN-USB Pro port 1 */
#define PCAN_USBBUS2    ((TPCANHandle)0x52u)   /* PCAN-USB Pro port 2 */
#define PCAN_USBBUS3    ((TPCANHandle)0x53u)
#define PCAN_USBBUS4    ((TPCANHandle)0x54u)
#define PCAN_USBBUS5    ((TPCANHandle)0x55u)
#define PCAN_USBBUS6    ((TPCANHandle)0x56u)
#define PCAN_USBBUS7    ((TPCANHandle)0x57u)
#define PCAN_USBBUS8    ((TPCANHandle)0x58u)

/* ── Baud rate constants ─────────────────────────────────────────────────── */
#define PCAN_BAUD_1M     ((TPCANBaudrate)0x0014u)
#define PCAN_BAUD_800K   ((TPCANBaudrate)0x0016u)
#define PCAN_BAUD_500K   ((TPCANBaudrate)0x001Cu)
#define PCAN_BAUD_250K   ((TPCANBaudrate)0x011Cu)   /* J1939 / KZValve */
#define PCAN_BAUD_125K   ((TPCANBaudrate)0x031Cu)
#define PCAN_BAUD_100K   ((TPCANBaudrate)0x432Fu)
#define PCAN_BAUD_50K    ((TPCANBaudrate)0x472Fu)
#define PCAN_BAUD_20K    ((TPCANBaudrate)0x532Fu)
#define PCAN_BAUD_10K    ((TPCANBaudrate)0x672Fu)
#define PCAN_BAUD_5K     ((TPCANBaudrate)0x7F7Fu)

/* ── Message type flags ──────────────────────────────────────────────────── */
#define PCAN_MESSAGE_STANDARD  ((TPCANMessageType)0x00u)  /* 11-bit standard */
#define PCAN_MESSAGE_RTR       ((TPCANMessageType)0x01u)  /* Remote frame    */
#define PCAN_MESSAGE_EXTENDED  ((TPCANMessageType)0x02u)  /* 29-bit extended */
#define PCAN_MESSAGE_FD        ((TPCANMessageType)0x04u)  /* FD frame        */
#define PCAN_MESSAGE_BRS       ((TPCANMessageType)0x08u)  /* FD bitrate-switch */
#define PCAN_MESSAGE_ESI       ((TPCANMessageType)0x10u)  /* FD error state  */
#define PCAN_MESSAGE_STATUS    ((TPCANMessageType)0x80u)  /* Error/status    */

/* ── Status/error codes ──────────────────────────────────────────────────── */
#define PCAN_ERROR_OK           ((TPCANStatus)0x00000u)  /* No error         */
#define PCAN_ERROR_XMTFULL      ((TPCANStatus)0x00001u)  /* TX buffer full   */
#define PCAN_ERROR_OVERRUN      ((TPCANStatus)0x00002u)  /* RX overrun       */
#define PCAN_ERROR_BUSLIGHT     ((TPCANStatus)0x00004u)  /* Bus error light  */
#define PCAN_ERROR_BUSHEAVY     ((TPCANStatus)0x00008u)  /* Bus error heavy  */
#define PCAN_ERROR_BUSWARNING   ((TPCANStatus)0x00008u)  /* Bus warning      */
#define PCAN_ERROR_BUSPASSIVE   ((TPCANStatus)0x40000u)  /* Bus passive      */
#define PCAN_ERROR_BUSOFF       ((TPCANStatus)0x00010u)  /* Bus off          */
#define PCAN_ERROR_ANYBUSERR    ((TPCANStatus)0x0001Cu)  /* Any bus error    */
#define PCAN_ERROR_QRCVEMPTY    ((TPCANStatus)0x00020u)  /* RX queue empty   */
#define PCAN_ERROR_QOVERRUN     ((TPCANStatus)0x00040u)  /* RX queue overrun */
#define PCAN_ERROR_QXMTFULL     ((TPCANStatus)0x00080u)  /* TX queue full    */
#define PCAN_ERROR_REGTEST      ((TPCANStatus)0x00100u)  /* Register test    */
#define PCAN_ERROR_NODRIVER     ((TPCANStatus)0x00200u)  /* Driver not loaded */
#define PCAN_ERROR_HWINUSE      ((TPCANStatus)0x00400u)  /* HW already used  */
#define PCAN_ERROR_NETINUSE     ((TPCANStatus)0x00800u)  /* Net already used */
#define PCAN_ERROR_ILLHW        ((TPCANStatus)0x01400u)  /* Invalid HW handle */
#define PCAN_ERROR_ILLNET       ((TPCANStatus)0x01800u)  /* Invalid net handle */
#define PCAN_ERROR_ILLCLIENT    ((TPCANStatus)0x01C00u)  /* Invalid client   */
#define PCAN_ERROR_INITIALIZE   ((TPCANStatus)0x04000u)  /* Not initialised  */
#define PCAN_ERROR_ILLOPERATION ((TPCANStatus)0x80000u)  /* Invalid op       */

/* ── Parameter IDs ───────────────────────────────────────────────────────── */
#define PCAN_DEVICE_ID          ((TPCANParameter)0x01u)
#define PCAN_5VOLTS_POWER       ((TPCANParameter)0x05u)
#define PCAN_RECEIVE_EVENT      ((TPCANParameter)0x06u)
#define PCAN_MESSAGE_FILTER     ((TPCANParameter)0x07u)
#define PCAN_API_VERSION        ((TPCANParameter)0x08u)
#define PCAN_CHANNEL_VERSION    ((TPCANParameter)0x09u)
#define PCAN_BUSOFF_AUTORESET   ((TPCANParameter)0x0Au)
#define PCAN_LISTEN_ONLY        ((TPCANParameter)0x0Bu)
#define PCAN_CHANNEL_CONDITION  ((TPCANParameter)0x10u)
#define PCAN_HARDWARE_NAME      ((TPCANParameter)0x11u)
#define PCAN_RECEIVE_STATUS     ((TPCANParameter)0x12u)

/* ── Filter modes ────────────────────────────────────────────────────────── */
#define PCAN_FILTER_CLOSE   ((TPCANMode)0x00u)
#define PCAN_FILTER_OPEN    ((TPCANMode)0x01u)
#define PCAN_FILTER_CUSTOM  ((TPCANMode)0x02u)

/* ── CAN message structure ───────────────────────────────────────────────── */
typedef struct {
    DWORD   ID;         /* 11 or 29-bit CAN message ID                      */
    BYTE    MSGTYPE;    /* Type of the message (PCAN_MESSAGE_* flags)        */
    BYTE    LEN;        /* Data Length Code (0..8)                           */
    BYTE    DATA[8];    /* Data bytes (up to 8)                              */
} TPCANMsg;

/* Timestamp returned alongside received messages */
typedef struct {
    DWORD   millis;             /* ms elapsed since CAN_Initialize          */
    WORD    millis_overflow;    /* overflow counter (increments at 2^32 ms) */
    WORD    micros;             /* us within current millisecond (0..999)   */
} TPCANTimestamp;

/* ── API function declarations ───────────────────────────────────────────── */
/* Resolved at link time via PCANBasic.lib / PCANBasic.dll.
 * Calling convention: __stdcall on Windows.                                */

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize a CAN channel.
 * For USB hardware: HwType=0, IOPort=0, Interrupt=0.                       */
TPCANStatus __stdcall CAN_Initialize(
    TPCANHandle   Channel,
    TPCANBaudrate Btr0Btr1,
    TPCANType     HwType,
    DWORD         IOPort,
    WORD          Interrupt);

/* Close and release a CAN channel */
TPCANStatus __stdcall CAN_Uninitialize(TPCANHandle Channel);

/* Reset a CAN channel (clears error counters) */
TPCANStatus __stdcall CAN_Reset(TPCANHandle Channel);

/* Get current status of a CAN channel */
TPCANStatus __stdcall CAN_GetStatus(TPCANHandle Channel);

/* Read a CAN message from the receive queue (non-blocking).
 * Returns PCAN_ERROR_QRCVEMPTY when no message is available.              */
TPCANStatus __stdcall CAN_Read(
    TPCANHandle     Channel,
    TPCANMsg       *MessageBuffer,
    TPCANTimestamp *TimestampBuffer);   /* NULL if timestamp not needed      */

/* Write a CAN message to the transmit queue */
TPCANStatus __stdcall CAN_Write(
    TPCANHandle  Channel,
    TPCANMsg    *MessageBuffer);

/* Set acceptance filter (FromID..ToID, extended or standard) */
TPCANStatus __stdcall CAN_FilterMessages(
    TPCANHandle Channel,
    DWORD       FromID,
    DWORD       ToID,
    TPCANMode   Mode);

/* Read a configuration parameter */
TPCANStatus __stdcall CAN_GetValue(
    TPCANHandle  Channel,
    TPCANParameter Parameter,
    void        *Buffer,
    DWORD        BufferLength);

/* Write a configuration parameter */
TPCANStatus __stdcall CAN_SetValue(
    TPCANHandle  Channel,
    TPCANParameter Parameter,
    void        *Buffer,
    DWORD        BufferLength);

/* Convert a status code to a human-readable string.
 * Language: 0x00=neutral, 0x09=English, 0x07=German, 0x0C=Italian       */
TPCANStatus __stdcall CAN_GetErrorText(
    TPCANStatus Error,
    WORD        Language,
    LPSTR       Buffer);

#ifdef __cplusplus
}
#endif
