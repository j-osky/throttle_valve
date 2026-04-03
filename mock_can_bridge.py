#!/usr/bin/env python3
"""
=============================================================================
mock_can_bridge.py — Josh Throttle Mock CAN Bridge
=============================================================================

PURPOSE
-------
This script is the third component of the bench test mock stack.  It sits
between tv_main.c (which talks to can0) and mock_daq.py (which serves
sensor data) and makes both of them think they are talking to real hardware.

WHAT IT SIMULATES
-----------------
1. TWO KZValve EH22 ACTUATORS (LOX at 0xBE, IPA at 0xBF)
   - Sends J1939 address claim frames at startup so tv_main's valve_init_sequence()
     finds both valves on the bus and advances to READY state.
   - Listens for Prop A2 absolute degree commands from tv_main.
   - Simulates the valve's PHYSICAL RESPONSE: the actual angle slews toward the
     commanded angle at 60 deg/s (the real EH22 1.5S max slew rate at full speed).
     This means a command to change angle by 10° takes 10/60 = 167ms to complete,
     just like the real valve.
   - Sends Prop A2 feedback frames back to tv_main every 100ms with:
       data[2] = simulated actual angle (degrees)  ← the process variable
       data[5] = FMI = 0 (no fault)                ← normal operation

2. PHYSICS MODEL COUPLING
   - After each slew step, pushes the simulated actual valve angles to
     mock_daq.py via POST /mock/valve_angles.
   - This causes mock_daq.py to recompute POM, PFM, PC from the new valve angles.
   - tv_main then reads these updated pressures, runs the PI controller,
     and sends new valve commands — completing the closed loop.

CLOSED-LOOP DATA FLOW (bench testing)
--------------------------------------
  tv_main (170 Hz)
    ├─ Reads POM/PFM/PC from mock_daq.py (HTTP)
    ├─ Runs tv_controller_2_1_step()
    └─ Every 17th tick (10 Hz): sends Prop A2 command to can0
           ↓ CAN frame on can0
  mock_can_bridge (this script)
    ├─ Parses commanded angle from Byte 1
    ├─ Slews simulated valve at 60 deg/s toward command
    ├─ Posts actual valve angles to mock_daq.py (HTTP, 10 Hz)
    │      ↓ HTTP POST /mock/valve_angles
    │  mock_daq.py recomputes POM/PFM/PC from new angles
    └─ Sends Prop A2 feedback frame back to can0
           ↓ CAN frame on can0
  tv_main receives feedback, logs actual position, checks FMI
  (loop continues...)

REQUIREMENTS
------------
  pip3 install python-can requests

USAGE
-----
  python3 mock_can_bridge.py [--iface can0] [--daq http://localhost:8050]

  Default interface is can0.  Use vcan0/can0 virtual interface for bench:
    sudo modprobe vcan
    sudo ip link add dev can0 type vcan
    sudo ip link set up can0
"""

import argparse
import math
import time
import threading
import requests
import can   # python-can library: pip3 install python-can

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

def is_prop_a2_cmd(msg) -> tuple | None:
    """
    Determine whether a received CAN message is a Prop A2 Absolute Mode command
    directed at one of our simulated valves.

    J1939 Prop A2 frame identification:
      - Must be extended frame (29-bit ID)
      - DP bit [24] = 1  (Data Page 1 = Prop A2 degrees mode)
      - PF = 0xEF [23:16] (PDU Format for Proprietary A/A2)
      - data[3] = 0x00    (Mode byte = Absolute position mode)
      - DLC = 8           (always 8 bytes for Prop A/A2)

    Returns:
      (dest_sa, src_sa) tuple if this is a valid absolute command, else None.
      dest_sa will be SA_LOX or SA_IPA if the command is for one of our valves.
    """
    if not msg.is_extended_id:
        return None   # Only handle 29-bit extended frames

    eid = msg.arbitration_id
    # Extract the relevant fields from the 29-bit EID
    pf  = (eid >> 16) & 0xFF   # PDU Format byte
    dp  = (eid >> 24) & 0x01   # Data Page bit (0=percent, 1=degrees)
    sa  = eid & 0xFF            # Source address (who sent it)
    da  = (eid >> 8) & 0xFF    # Destination address (PS byte for PDU1)

    if pf == 0xEF and dp == 1 and len(msg.data) == 8:
        mode = msg.data[3]     # Byte 4 = mode
        if mode == 0x00:       # 0x00 = Absolute position mode
            return da, sa
    return None


# =============================================================================
# FRAME BUILDER: build_prop_a2_feedback
# =============================================================================

def build_prop_a2_feedback(src_sa: int, pi_sa: int,
                            actual_deg: float, cmd_deg: float,
                            speed_pct: int) -> can.Message:
    """
    Build a Prop A2 position feedback frame that mimics what the real EH22
    valve sends back to tv_main (KZValve manual Section 8.1).

    This is the response format the valve uses for BOTH:
      - Replies to Request PGN
      - Unsolicited periodic broadcasts (after periodic_cfg is configured)

    Wire format (8 bytes, Prop A2 direction valve→Pi):
      Byte 1 (data[0]): Commanded position echo in degrees
                         (what the valve thinks Pi last commanded — echo back)
      Byte 2 (data[1]): Commanded motor speed echo (%)
      Byte 3 (data[2]): ACTUAL current position in degrees
                         *** This is the process variable tv_main reads ***
                         tv_main's PI controller uses this to compute error.
      Byte 4 (data[3]): 0xFF (mode, not meaningful in response)
      Byte 5 (data[4]): Measured supply voltage, rounded to nearest Volt
                         (we always report 24V for the mock)
      Byte 6 (data[5]): FMI — Fault Mode Indicator
                         0 = no fault (what mock always reports)
                         See kzvalve_can.h FMI_* constants for fault values
      Bytes 7-8:         0xFF (reserved)

    The EID is constructed with the valve as source (src_sa) and Pi as dest:
      EID = (6<<26)|(0<<25)|(1<<24)|(0xEF<<16)|(pi_sa<<8)|src_sa
          = 0x19EF0100 | (pi_sa<<8) | src_sa  (for Pi=0x01)
    """
    eid = (6 << 26) | (0 << 25) | (1 << 24) | (0xEF << 16) | (pi_sa << 8) | src_sa

    data = bytearray(8)
    data[0] = int(max(0, min(255, cmd_deg)))    # Byte 1: commanded angle echo
    data[1] = int(max(50, min(100, speed_pct))) # Byte 2: speed echo
    data[2] = int(max(0, min(255, actual_deg))) # Byte 3: ACTUAL position (deg)
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

    def __init__(self, iface: str, daq_url: str):
        self.iface   = iface
        self.daq_url = daq_url.rstrip("/")
        self.bus     = None

        # Start both valves at the Simulink 500 lbf initial conditions
        # (matching THETA_O and THETA_F in tv_main.c)
        self.lox_actual = 41.06
        self.ipa_actual = 40.68
        self.lox_cmd    = 41.06
        self.ipa_cmd    = 40.68

        self.last_t = time.monotonic()

    def connect(self):
        """Open the SocketCAN interface via python-can."""
        self.bus = can.interface.Bus(channel=self.iface, bustype="socketcan")
        print(f"[Bridge] Connected to CAN interface: {self.iface}")

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
        Transmit Prop A2 position feedback frames for both simulated valves.
        These frames flow from valve → Pi (src=valve SA, dest=Pi SA).
        tv_main's can_process_rx() receives them and updates:
          g_state.lox_actual_deg, g_state.ipa_actual_deg, g_state.lox_fmi, ...
        """
        lox_fb = build_prop_a2_feedback(SA_LOX, SA_PI,
                                         self.lox_actual, self.lox_cmd, 100)
        ipa_fb = build_prop_a2_feedback(SA_IPA, SA_PI,
                                         self.ipa_actual, self.ipa_cmd, 100)
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

        # Background thread: feedback and physics update at 10 Hz (100ms period)
        def feedback_loop():
            while True:
                self.slew_valves()    # Advance valve positions at 60 deg/s
                self.push_to_daq()   # Update mock_daq.py physics
                self.send_feedback() # Send Prop A2 feedback to tv_main on CAN
                time.sleep(0.1)      # 100ms = 10 Hz

        threading.Thread(target=feedback_loop, daemon=True).start()

        print(f"[Bridge] Listening for Prop A2 commands on {self.iface}...")
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
            deg    = msg.data[0]   # Byte 1: commanded position in degrees
            # msg.data[1] = speed (we ignore — slew rate is fixed at SLEW_RATE)

            # Update the commanded angle for the appropriate valve
            if da == SA_LOX:
                self.lox_cmd = float(deg)
            elif da == SA_IPA:
                self.ipa_cmd = float(deg)
            # Ignore commands addressed to other devices


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Josh Throttle Mock CAN Bridge")
    ap.add_argument("--iface", default="can0",
                    help="SocketCAN interface name (default: can0)")
    ap.add_argument("--daq",   default="http://localhost:8050",
                    help="mock_daq.py base URL (default: http://localhost:8050)")
    args = ap.parse_args()

    bridge = MockCANBridge(args.iface, args.daq)
    bridge.run()
