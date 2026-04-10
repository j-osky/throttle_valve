#!/usr/bin/env python3
"""
=============================================================================
mock_can_bridge.py — Josh Throttle Mock CAN Bridge  (Linux + Windows)
=============================================================================

PURPOSE
-------
This script is the bench test mock stack CAN bridge.  It sits between
tv_main_propa_win.exe (or tv_main_propa on Linux) and mock_daq.py, making
both think they are talking to real KZValve EH22 actuators.

WHAT IT SIMULATES
-----------------
1. TWO KZValve EH22 ACTUATORS (LOX at 0xBE, IPA at 0xBF)
   - Sends J1939 address claim frames at startup so tv_main's init sequence
     finds both valves on the bus and advances to READY state.
   - Listens for Prop A (percent) or Prop A2 (degree) commands.
   - Simulates the valve PHYSICAL RESPONSE: slews at 60 deg/s toward command.
   - Sends feedback frames every 100ms with actual position and FMI=0.

2. PHYSICS MODEL COUPLING
   - Pushes simulated valve angles to mock_daq.py at 170 Hz.
   - mock_daq.py recomputes POM/PFM/PC from these angles.
   - tv_main reads updated pressures and closes the loop.

REQUIREMENTS
------------
  pip install python-can requests

  For Windows with PCAN-USB Pro (no virtual CAN needed):
    pip install python-can[pcan]
    Install PEAK drivers: PCAN_USB_Setup.exe from peak-system.com

  For Linux (virtual CAN bench):
    sudo modprobe vcan && sudo ip link add dev can0 type vcan && sudo ip link set up can0

USAGE
-----
  # Linux virtual CAN (bench):
  python3 mock_can_bridge.py --iface can0

  # Windows PCAN-USB Pro (bench):
  python3 mock_can_bridge.py --iface PCAN_USBBUS1 --backend pcan

  # Windows second port:
  python3 mock_can_bridge.py --iface PCAN_USBBUS2 --backend pcan
"""

import argparse
import math
import time
import threading
import requests
import can
from typing import Optional, Tuple   # python-can library: pip3 install python-can

# =============================================================================
# J1939 ADDRESS CONSTANTS
# Must match KZVALVE_SA_LOX, KZVALVE_SA_IPA, KZVALVE_SA_PI in kzvalve_can.h
# =============================================================================
SA_LOX = 0xBE   # LOX valve source address (190)
SA_IPA = 0xBF   # IPA valve source address (191)
SA_PI  = 0x01   # Pi ECU source address (1)

# =============================================================================
# FRAME DETECTION: is_prop_a2_cmd
# =============================================================================

def is_prop_a2_cmd(msg) -> Optional[Tuple[int, int]]:
    """
    Determine whether a received CAN message is a Prop A or Prop A2 Absolute
    Mode command directed at one of our simulated valves.

    Accepts both:
      Prop A  (DP=0, PGN 0x00EF00): percent of full open — used by tv_main_propa
      Prop A2 (DP=1, PGN 0x01EF00): degrees              — used by tv_main

    Common requirements:
      - Extended frame (29-bit ID)
      - PF = 0xEF
      - data[3] = 0x00 (Absolute position mode)
      - DLC = 8

    Returns:
      (dest_sa, src_sa) tuple if this is a valid absolute command, else None.
    """
    if not msg.is_extended_id:
        return None

    eid = msg.arbitration_id
    pf  = (eid >> 16) & 0xFF
    sa  = eid & 0xFF
    da  = (eid >> 8) & 0xFF

    # Accept both DP=0 (Prop A percent) and DP=1 (Prop A2 degrees)
    if pf == 0xEF and len(msg.data) == 8:
        mode = msg.data[3]
        if mode == 0x00:   # Absolute position mode
            return da, sa
    return None


# =============================================================================
# FRAME BUILDER: build_prop_a2_feedback
# =============================================================================

def build_prop_feedback(src_sa: int, pi_sa: int,
                         actual_deg: float, cmd_deg: float,
                         speed_pct: int, prop_a: bool = False) -> can.Message:
    """
    Build a Prop A or Prop A2 position feedback frame.

    prop_a=False (default): Prop A2 (DP=1) — degrees mode
      data[0]: commanded angle in degrees
      data[2]: actual position in degrees

    prop_a=True: Prop A (DP=0) — percent mode
      data[0]: commanded position as percent (0-100)
      data[2]: actual position as percent (0-100)

    Both use the same EID structure with the appropriate DP bit.
    """
    dp  = 0 if prop_a else 1
    eid = (6 << 26) | (0 << 25) | (dp << 24) | (0xEF << 16) | (pi_sa << 8) | src_sa

    if prop_a:
        cmd_val    = int(max(0, min(100, round(cmd_deg    / 90.0 * 100.0))))
        actual_val = int(max(0, min(100, round(actual_deg / 90.0 * 100.0))))
    else:
        cmd_val    = int(max(0, min(255, cmd_deg)))
        actual_val = int(max(0, min(255, actual_deg)))

    data = bytearray(8)
    data[0] = cmd_val                            # Byte 1: commanded position echo
    data[1] = int(max(50, min(100, speed_pct)))  # Byte 2: speed echo
    data[2] = actual_val                         # Byte 3: ACTUAL position
    data[3] = 0xFF                               # Byte 4: mode (0xFF in response)
    data[4] = 24                                 # Byte 5: 24V supply voltage
    data[5] = 0                                  # Byte 6: FMI = 0 (no fault)
    data[6] = 0xFF                               # Byte 7: reserved
    data[7] = 0xFF                               # Byte 8: reserved

    return can.Message(
        arbitration_id=eid,
        data=bytes(data),
        is_extended_id=True,
        is_remote_frame=False
    )

# Keep old name as alias for Prop A2 (backward compat with any callers)
def build_prop_a2_feedback(src_sa, pi_sa, actual_deg, cmd_deg, speed_pct):
    return build_prop_feedback(src_sa, pi_sa, actual_deg, cmd_deg, speed_pct, prop_a=False)


# =============================================================================
# MOCK CAN BRIDGE CLASS
# =============================================================================

class MockCANBridge:
    """
    Simulates two KZValve EH22 actuators on a CAN bus.

    Internal state:
      lox_cmd / ipa_cmd     — Last commanded angle from tv_main (degrees)
      lox_actual / ipa_actual — Simulated current physical angle (degrees)
                                Updated by slew_valves() at 60 deg/s
    """

    # Physical slew rate of the EH22 1.5S at full motor speed (100%)
    # = 90 degrees / 1.5 seconds = 60 degrees per second
    SLEW_RATE_DEG_PER_SEC = 60.0

    def __init__(self, iface: str, daq_url: str, backend: str = "socketcan"):
        self.iface   = iface
        self.daq_url = daq_url.rstrip("/")
        self.backend = backend
        self.bus     = None

        # Start valves at 0 deg (closed/unknown).
        # The operator must press Init ICs in the GUI before firing —
        # this ensures actual positions reflect reality, not an assumed IC.
        self.lox_actual = 0.0
        self.ipa_actual = 0.0
        self.lox_cmd    = 0.0
        self.ipa_cmd    = 0.0
        # Track whether each valve is being commanded in Prop A (percent) mode
        self.lox_prop_a = False
        self.ipa_prop_a = False

        self.last_t = time.monotonic()

    def connect(self):
        """Open the CAN interface via python-can.

        On Linux: uses socketcan backend (can0, can1, vcan0, etc.)
        On Windows: uses pcan backend (PCAN_USBBUS1, PCAN_USBBUS2, etc.)

        The backend is set by --backend argument (default: socketcan on Linux,
        pcan on Windows).  python-can handles the difference transparently.
        """
        kwargs = {"channel": self.iface, "interface": self.backend,
                  "bitrate": 250000}
        # socketcan does not need bitrate (set on the interface), pcan does
        if self.backend == "socketcan":
            del kwargs["bitrate"]
        self.bus = can.interface.Bus(**kwargs)
        print(f"[Bridge] Connected: interface={self.backend} channel={self.iface}")

    def slew_valves(self):
        """
        Advance actual valve angles toward commanded angles at the real-world
        slew rate (60 deg/s).

        This is called every 100ms.  Each call can move the valve up to:
          max_step = 60 deg/s * 0.1 s = 6 degrees per call

        If the remaining distance to command is <= max_step, snap to the command.
        Otherwise move by max_step in the correct direction.

        This simulates the physical delay of the valve motor responding —
        tv_main should not expect instantaneous response.
        """
        now      = time.monotonic()
        dt       = now - self.last_t   # time since last slew update (seconds)
        self.last_t = now
        max_step = self.SLEW_RATE_DEG_PER_SEC * dt   # max degrees this step

        def step(actual: float, cmd: float) -> float:
            diff = cmd - actual
            if abs(diff) <= max_step:
                return cmd   # close enough — snap to target
            return actual + math.copysign(max_step, diff)  # move toward target

        self.lox_actual = step(self.lox_actual, self.lox_cmd)
        self.ipa_actual = step(self.ipa_actual, self.ipa_cmd)

    def push_to_daq(self):
        """
        Send current actual valve angles to mock_daq.py so the physics model
        updates its flow calculation.  This is what makes the sensor pressures
        (POM, PFM, PC) respond to valve movements in the simulation.

        Uses a short timeout to avoid blocking the feedback thread if mock_daq
        is slow or unreachable.
        """
        try:
            requests.post(
                f"{self.daq_url}/mock/valve_angles",
                json={"lox_deg": self.lox_actual, "ipa_deg": self.ipa_actual},
                timeout=0.05   # 50ms max — must not block the 100ms loop
            )
        except Exception:
            pass   # Silently ignore — mock_daq may not be running yet

    def send_feedback(self):
        """
        Transmit position feedback frames for both simulated valves.
        Uses Prop A (percent) or Prop A2 (degrees) to match the command mode.
        """
        lox_fb = build_prop_feedback(SA_LOX, SA_PI,
                                      self.lox_actual, self.lox_cmd, 100,
                                      prop_a=self.lox_prop_a)
        ipa_fb = build_prop_feedback(SA_IPA, SA_PI,
                                      self.ipa_actual, self.ipa_cmd, 100,
                                      prop_a=self.ipa_prop_a)
        try:
            self.bus.send(lox_fb)
            self.bus.send(ipa_fb)
        except Exception:
            pass

    def send_address_claims(self):
        """
        Broadcast J1939 Address Claimed messages for both simulated valves.

        tv_main's valve_init_sequence() waits for address claim frames from
        0xBE (LOX) and 0xBF (IPA) before advancing to READY state.  Without
        these frames tv_main would wait 5 seconds and print a warning.

        J1939 Address Claimed frame:
          PGN: 0xEE00 (PF=0xEE, PS=0xFF global broadcast)
          EID: (6<<26)|(0xEE<<16)|(0xFF<<8)|SA
          Data: 8 bytes of the device's 64-bit NAME
                Bit 63 (data[7] bit 7) = Arbitrary Address Capable = 1
        """
        for sa in [SA_LOX, SA_IPA]:
            eid  = (6 << 26) | (0xEE << 16) | (0xFF << 8) | sa
            data = bytearray(8)
            data[7] = 0xA0   # Industry Group=2 (bits[6:4]=010), Arbitrary=1 (bit7)
            msg = can.Message(arbitration_id=eid, data=bytes(data),
                              is_extended_id=True)
            try:
                self.bus.send(msg)
            except Exception:
                pass
        print(f"[Bridge] Address claims sent: LOX=0x{SA_LOX:02X}, IPA=0x{SA_IPA:02X}")

    def run(self):
        """
        Main execution loop.

        1. Connect to CAN bus.
        2. Wait 300ms for tv_main to send its address claim, then send ours.
        3. Start a background thread that every 100ms:
             - Slews valve angles toward commanded positions
             - Pushes angles to mock_daq.py physics model
             - Sends Prop A2 feedback frames to tv_main
        4. Main thread: listen for Prop A2 absolute commands from tv_main,
           update lox_cmd / ipa_cmd when received.
        """
        self.connect()
        time.sleep(0.3)   # Give tv_main time to send its own address claim first
        self.send_address_claims()

        # Physics update loop at 170 Hz — matches Simulink plant update rate.
        # Slewing at 170 Hz means the DAQ sees a smooth pressure ramp as the
        # valve moves, identical to how the Simulink 2_0 plant behaved.
        # Without this, the DAQ holds pressure constant for 100ms then steps,
        # which causes the FIR-filtered MR signal to lag and the IPA integrator
        # to wind up, producing oscillation that doesn't exist in Simulink.
        PHYSICS_HZ  = 170
        PHYSICS_DT  = 1.0 / PHYSICS_HZ          # 5.88 ms
        CAN_DIVIDER = PHYSICS_HZ // 10           # Send CAN feedback every 17 ticks = 10 Hz
        SLEW_STEP   = self.SLEW_RATE_DEG_PER_SEC * PHYSICS_DT  # 0.353 deg/tick

        def feedback_loop():
            tick = 0
            while True:
                t0 = time.monotonic()

                # Slew valves one physics step toward commanded angle
                def step_valve(actual, cmd):
                    diff = cmd - actual
                    if abs(diff) <= SLEW_STEP:
                        return cmd
                    return actual + math.copysign(SLEW_STEP, diff)

                self.lox_actual = step_valve(self.lox_actual, self.lox_cmd)
                self.ipa_actual = step_valve(self.ipa_actual, self.ipa_cmd)

                # Update DAQ physics every tick at 170 Hz
                self.push_to_daq()

                # Send CAN feedback at 10 Hz (every 17 ticks)
                tick += 1
                if tick >= CAN_DIVIDER:
                    tick = 0
                    self.send_feedback()

                # Sleep for remainder of 5.88ms tick
                elapsed = time.monotonic() - t0
                sleep_t = max(0.0, PHYSICS_DT - elapsed)
                time.sleep(sleep_t)

        threading.Thread(target=feedback_loop, daemon=True).start()

        print(f"[Bridge] Listening for Prop A / Prop A2 commands on {self.iface} ({self.backend})...")
        print(f"[Bridge] Will relay valve angles to {self.daq_url}")

        # Main thread: receive and parse CAN frames
        while True:
            msg = self.bus.recv(timeout=1.0)   # Block up to 1s waiting for a frame
            if msg is None:
                continue   # Timeout — no frame received, loop again

            # Check if this is a Prop A2 absolute command directed at our valves
            result = is_prop_a2_cmd(msg)
            if result is None:
                continue   # Not a command we care about — ignore

            da, sa = result    # Destination address, source address

            # Byte 1 (data[0]) meaning depends on DP bit in the EID:
            #   DP=1 (Prop A2, 0x19EF...): value is degrees  (0-90)
            #   DP=0 (Prop A,  0x18EF...): value is percent  (0-100) → convert to degrees
            eid     = msg.arbitration_id
            dp_bit  = (eid >> 24) & 0x1
            prop_a  = (dp_bit == 0)
            raw_val = float(msg.data[0])
            deg     = raw_val if not prop_a else raw_val * 90.0 / 100.0

            # Apply a 0.5 deg deadband: only update cmd if the new value differs
            # by more than 0.5 deg from current.  This prevents 1-count integer
            # dithering from the uint8 quantization in can_send_valve_commands()
            # from continuously slewing the simulated valve back and forth.
            DEADBAND = 0.5
            if da == SA_LOX:
                self.lox_prop_a = prop_a
                if abs(deg - self.lox_cmd) > DEADBAND:
                    self.lox_cmd = deg
            elif da == SA_IPA:
                self.ipa_prop_a = prop_a
                if abs(deg - self.ipa_cmd) > DEADBAND:
                    self.ipa_cmd = deg
            # Ignore commands addressed to other devices


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    import platform
    default_backend = "pcan" if platform.system() == "Windows" else "socketcan"
    default_iface   = "PCAN_USBBUS1" if platform.system() == "Windows" else "can0"

    ap = argparse.ArgumentParser(description="Josh Throttle Mock CAN Bridge")
    ap.add_argument("--iface",   default=default_iface,
                    help=f"CAN interface (default: {default_iface}). "
                         "Linux: can0/can1/vcan0. Windows: PCAN_USBBUS1/PCAN_USBBUS2")
    ap.add_argument("--daq",     default="http://localhost:8050",
                    help="mock_daq.py base URL (default: http://localhost:8050)")
    ap.add_argument("--backend", default=default_backend,
                    help=f"python-can backend (default: {default_backend}). "
                         "Linux: socketcan. Windows: pcan")
    args = ap.parse_args()

    bridge = MockCANBridge(args.iface, args.daq, backend=args.backend)
    bridge.run()