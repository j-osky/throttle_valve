#!/usr/bin/env python3
"""
mock_pt.py - Bench PT-board simulator for the no-DAQ controller.

Listens for valve commands on CAN, applies the same 60 deg/s slew used by the
old DAQ push path, runs the Moonshine 2 pressure physics from mock_daq.py, and
broadcasts POM/PFM/PC together in one proprietary CAN frame:

  PGN  0x00FF50
  SA   0xA5
  DLC  8
  data[0:1] = POM psi * 10, little-endian uint16
  data[2:3] = PFM psi * 10, little-endian uint16
  data[4:5] = PC  psi * 10, little-endian uint16
  data[6]   = sequence counter
  data[7]   = reserved
"""

import argparse
import math
import platform
import struct
import time

import can


SA_LOX = 0xBE
SA_IPA = 0xBF
PT_SENSOR_SA = 0xA5
SENSOR_EID_BASE = 0x18FF5000
SLEW_RATE_DEG_PER_SEC = 60.0
TICK_HZ = 170.0
PSI_SCALE = 10.0

CV_BP = [0, 13.5, 18, 27, 36, 45, 54, 63, 72, 81, 90]
CV_VAL = [0, 0.1, 0.2, 0.5, 0.7, 1.1, 1.8, 2.4, 3.3, 4.5, 5.4]

RHO_LOX = 1090.8958
RHO_IPA = 785.0926
SG_LOX = RHO_LOX / 1000.0
SG_IPA = RHO_IPA / 1000.0
CDA_LOX = 2.58e-5
CDA_IPA = 2.53e-5
K_CV = 7.598e-7
V_E_MS = 1752.461343
K_PC = 1753390.10
PSI2PA = 6894.757
P_TANK_PSI = 500.0

THETA_O = 41.06017851
THETA_F = 40.67697413


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def deg_to_cv(deg: float) -> float:
    deg = clamp(float(deg), 0.0, 90.0)
    if deg <= CV_BP[0]:
        return CV_VAL[0]
    if deg >= CV_BP[-1]:
        return CV_VAL[-1]
    for i in range(len(CV_BP) - 1):
        if CV_BP[i] <= deg <= CV_BP[i + 1]:
            frac = (deg - CV_BP[i]) / (CV_BP[i + 1] - CV_BP[i])
            return CV_VAL[i] + frac * (CV_VAL[i + 1] - CV_VAL[i])
    return 0.0


def encode_psi(psi: float) -> int:
    return int(clamp(round(psi * PSI_SCALE), 0, 0xFFFF))


def decode_command_deg(msg: can.Message):
    if not msg.is_extended_id or len(msg.data) != 8:
        return None

    eid = msg.arbitration_id
    dp = (eid >> 24) & 0x1
    pf = (eid >> 16) & 0xFF
    da = (eid >> 8) & 0xFF

    if pf != 0xEF or msg.data[3] != 0x00:
        return None

    raw = msg.data[0]
    if dp == 0:
        deg = raw * 90.0 / 100.0
    else:
        deg = float(raw)
    return da, clamp(deg, 0.0, 90.0)


class Physics:
    def __init__(self):
        self.lox_deg = THETA_O
        self.ipa_deg = THETA_F
        self.pc_psi = 322.8241
        self.pom_psi = 370.7493
        self.pfm_psi = 370.8182
        self.mdot_lox = 0.692724
        self.mdot_ipa = 0.576699
        self.thrust_lbf = 500.14
        self.mr = 1.2012

    def set_valves(self, lox_deg: float, ipa_deg: float):
        self.lox_deg = clamp(lox_deg, 0.0, 90.0)
        self.ipa_deg = clamp(ipa_deg, 0.0, 90.0)

    def step(self):
        cv_lox = deg_to_cv(self.lox_deg)
        cv_ipa = deg_to_cv(self.ipa_deg)
        p_tank = P_TANK_PSI * PSI2PA
        pc_pa = self.pc_psi * PSI2PA if self.pc_psi > 0 else PSI2PA * 100.0

        a_lox = K_CV ** 2 * RHO_LOX * cv_lox ** 2 * p_tank / SG_LOX
        b_lox = K_CV ** 2 * RHO_LOX * cv_lox ** 2 / SG_LOX
        a_ipa = K_CV ** 2 * RHO_IPA * cv_ipa ** 2 * p_tank / SG_IPA
        b_ipa = K_CV ** 2 * RHO_IPA * cv_ipa ** 2 / SG_IPA

        for _ in range(20):
            pom_pa = (a_lox + 2 * CDA_LOX ** 2 * pc_pa) / (2 * CDA_LOX ** 2 + b_lox)
            pfm_pa = (a_ipa + 2 * CDA_IPA ** 2 * pc_pa) / (2 * CDA_IPA ** 2 + b_ipa)
            dp_lox = max(0.0, pom_pa - pc_pa)
            dp_ipa = max(0.0, pfm_pa - pc_pa)
            mdot_lox = CDA_LOX * math.sqrt(2.0 * RHO_LOX * dp_lox) if dp_lox > 0 else 0.0
            mdot_ipa = CDA_IPA * math.sqrt(2.0 * RHO_IPA * dp_ipa) if dp_ipa > 0 else 0.0
            pc_new = K_PC * (mdot_lox + mdot_ipa) if (mdot_lox + mdot_ipa) > 0 else 0.0
            pc_blended = 0.5 * pc_pa + 0.5 * pc_new
            if abs(pc_blended - pc_pa) / (pc_pa + 1.0) < 1e-6:
                pc_pa = pc_blended
                break
            pc_pa = pc_blended

        pom_pa = (a_lox + 2 * CDA_LOX ** 2 * pc_pa) / (2 * CDA_LOX ** 2 + b_lox)
        pfm_pa = (a_ipa + 2 * CDA_IPA ** 2 * pc_pa) / (2 * CDA_IPA ** 2 + b_ipa)
        dp_lox = max(0.0, pom_pa - pc_pa)
        dp_ipa = max(0.0, pfm_pa - pc_pa)
        mdot_lox = CDA_LOX * math.sqrt(2.0 * RHO_LOX * dp_lox) if dp_lox > 0 else 0.0
        mdot_ipa = CDA_IPA * math.sqrt(2.0 * RHO_IPA * dp_ipa) if dp_ipa > 0 else 0.0
        mdot_tot = mdot_lox + mdot_ipa

        self.pom_psi = pom_pa / PSI2PA
        self.pfm_psi = pfm_pa / PSI2PA
        self.pc_psi = pc_pa / PSI2PA
        self.mdot_lox = mdot_lox
        self.mdot_ipa = mdot_ipa
        self.thrust_lbf = mdot_tot * V_E_MS * 0.22482014388489208
        self.mr = mdot_lox / mdot_ipa if mdot_ipa > 1e-6 else 0.0


class MockPTBoard:
    def __init__(self, iface: str, backend: str):
        kwargs = {"channel": iface, "interface": backend, "bitrate": 250000}
        if backend == "socketcan":
            del kwargs["bitrate"]
        self.bus = can.interface.Bus(**kwargs)
        self.physics = Physics()
        self.lox_cmd_deg = THETA_O
        self.ipa_cmd_deg = THETA_F
        self.lox_sim_deg = THETA_O
        self.ipa_sim_deg = THETA_F
        self.last_t = time.monotonic()
        self.seq = 0
        self.iface = iface
        self.backend = backend

    def drain_commands(self):
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return
            decoded = decode_command_deg(msg)
            if decoded is None:
                continue
            dest_sa, deg = decoded
            if dest_sa == SA_LOX:
                self.lox_cmd_deg = deg
            elif dest_sa == SA_IPA:
                self.ipa_cmd_deg = deg

    def slew_valves(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_t)
        self.last_t = now
        max_step = SLEW_RATE_DEG_PER_SEC * dt

        def step(actual: float, target: float) -> float:
            diff = target - actual
            if abs(diff) <= max_step:
                return target
            return actual + math.copysign(max_step, diff)

        self.lox_sim_deg = clamp(step(self.lox_sim_deg, self.lox_cmd_deg), 0.0, 90.0)
        self.ipa_sim_deg = clamp(step(self.ipa_sim_deg, self.ipa_cmd_deg), 0.0, 90.0)

    def send_sensor_frame(self):
        data = struct.pack(
            "<HHHBB",
            encode_psi(self.physics.pom_psi),
            encode_psi(self.physics.pfm_psi),
            encode_psi(self.physics.pc_psi),
            self.seq & 0xFF,
            0,
        )
        msg = can.Message(
            arbitration_id=SENSOR_EID_BASE | PT_SENSOR_SA,
            is_extended_id=True,
            data=data,
        )
        self.bus.send(msg)
        self.seq = (self.seq + 1) & 0xFF

    def run(self):
        print(f"[MockPT] Connected on {self.iface} ({self.backend})")
        print("[MockPT] Publishing PGN 0x00FF50 with POM/PFM/PC in one CAN frame")
        next_tick = time.monotonic()
        period = 1.0 / TICK_HZ

        while True:
            self.drain_commands()
            self.slew_valves()
            self.physics.set_valves(self.lox_sim_deg, self.ipa_sim_deg)
            self.physics.step()
            self.send_sensor_frame()

            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()


if __name__ == "__main__":
    default_backend = "pcan" if platform.system() == "Windows" else "socketcan"
    default_iface = "PCAN_USBBUS1" if platform.system() == "Windows" else "can0"

    ap = argparse.ArgumentParser(description="Josh Throttle mock PT board")
    ap.add_argument("--iface", default=default_iface,
                    help=f"CAN interface (default: {default_iface})")
    ap.add_argument("--backend", default=default_backend,
                    help=f"python-can backend (default: {default_backend})")
    args = ap.parse_args()

    try:
        MockPTBoard(args.iface, args.backend).run()
    except KeyboardInterrupt:
        pass
