#!/usr/bin/env python3
"""
=============================================================================
mock_daq.py — Josh Throttle Mock DAQ Server
=============================================================================

PURPOSE
-------
This script is a DROP-IN REPLACEMENT for the real DAQstra instance during
bench testing.  It serves the same REST API endpoints that tv_main.c calls
to read pressure sensor values, but instead of reading real hardware it
computes the values from a physics simulation of the Moonshine 2 propellant
system.

This means you can run the FULL CLOSED LOOP on the bench:
  tv_main → reads pressures from this mock → computes valve commands →
  sends CAN → mock_can_bridge intercepts → updates this mock's valve angles →
  pressures change → tv_main reacts → (repeat)

PHYSICS MODEL
-------------
Implements the exact flow equations from tv_controller_2_0.slx (LOX_flow / LOX_mdot
MATLAB Function blocks), matched to the Simulink-generated tv_controller_2_1.c constants.

1. VALVE Cv: Lookup table identical to the Simulink "deg → Cv" block.

2. MANIFOLD PRESSURE (Cv-based LOX_flow equation from Simulink 2_0):
   Numerically solves for p_m given Cv, Pc, and tank pressure:
     p_m = (k² * rho * Cv² * p_tank/SG  +  2 * CdA² * p_c)
           / (2 * CdA²  +  k² * rho * Cv² / SG)
   where k = 7.598e-7 m², the Cv-to-area proportionality constant.

3. MASS FLOW (from same LOX_flow / IPA_flow block):
   mdot = CdA * sqrt(2 * rho * max(p_m - p_c, 0))   [kg/s]

4. CHAMBER PRESSURE (iterated fixed-point, damped for stability):
   Pc_new = (mdot_lox + mdot_ipa) * V_e / (At * ... )
   Iterates until converged. Uses damped update to prevent oscillation.
   NOTE: The mock does not model combustion — Pc is back-calculated from
   mdot and v_e so that the mdot/thrust estimates are self-consistent.

5. THRUST ESTIMATE (matches Simulink Gain7+Gain8 block):
   thrust_lbf = (mdot_lox + mdot_ipa) * V_E_MS * 0.22482

DAQSTRA API COMPATIBILITY
--------------------------
tv_main.c calls these endpoints:
  GET /api/v1/sensors/<sensor_id>   → returns {"metrics": {"latest_value": N}}
  GET /api/v1/sensors               → lists all sensors
  GET /health                       → service health check

The sensor IDs must EXACTLY MATCH the SENSOR_ID_* defines in tv_main.c.
Default mapping:
  b1_log_data_ads1256%231  →  POM (LOX manifold pressure, psi)
  b1_log_data_ads1256%232  →  PFM (IPA manifold pressure, psi)
  b1_log_data_ads1256%233  →  PC  (chamber pressure, psi)

MOCK-ONLY ENDPOINTS
-------------------
  GET  /mock/status           → full physics snapshot (for debugging)
  POST /mock/valve_angles     → {"lox_deg": N, "ipa_deg": N}
                                Updates the simulated valve positions.
                                Called by mock_can_bridge.py at 10 Hz.

USAGE
-----
  python3 mock_daq.py [--port 8050]
  python3 mock_daq.py --port 8050  # default port matches DAQstra
"""

import argparse
import math
import time
import threading
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, unquote

# =============================================================================
# VALVE Cv LOOKUP TABLE
# =============================================================================
# This is identical to the "deg -> Cv (Valve position -> Cv)" Simulink block.
# Breakpoints are valve angle in degrees; values are flow coefficient Cv.
# Cv is dimensionless in this context — it scales the CdA orifice area.
# Note the nonlinearity: most of the flow gain happens in the 45-90° range.

CV_BP  = [0,   13.5, 18,  27,  36,  45,  54,  63,  72,  81,  90 ]  # degrees
CV_VAL = [0,   0.1,  0.2, 0.5, 0.7, 1.1, 1.8, 2.4, 3.3, 4.5, 5.4]  # Cv

def deg_to_cv(deg: float) -> float:
    """
    Linearly interpolate Cv from valve angle using the lookup table above.
    Clamps to [0, 90] degrees before interpolation.
    Returns 0.0 for a fully closed valve (0°), 5.4 for fully open (90°).
    """
    deg = max(0.0, min(90.0, float(deg)))
    if deg <= CV_BP[0]:  return CV_VAL[0]
    if deg >= CV_BP[-1]: return CV_VAL[-1]
    for i in range(len(CV_BP) - 1):
        if CV_BP[i] <= deg <= CV_BP[i + 1]:
            frac = (deg - CV_BP[i]) / (CV_BP[i + 1] - CV_BP[i])
            return CV_VAL[i] + frac * (CV_VAL[i + 1] - CV_VAL[i])
    return 0.0

# =============================================================================
# PROPELLANT PHYSICAL CONSTANTS  (matched to Simulink tv_controller_2_1 / tv_controller_2_0)
# =============================================================================
# All constants derived from the Simulink-generated C code and gain_scheduling.m.
# The flow model uses the Cv-based LOX_flow / LOX_mdot functions from tv_controller_2_0.slx.

# Densities — extracted from Simulink mdot constants:
#   lox_mdot = CdA_lox * sqrt(dP_psi * 2 * rho_lox * PSI2PA)
#   constant in code: 1.504292293924e7 = 2 * rho_lox * 6894.757
RHO_LOX   = 1090.8958   # kg/m³  (from Simulink: 1.504292293924e7 / 2 / 6894.757)
RHO_IPA   =  785.0926   # kg/m³  (from Simulink: 1.08260448331e7  / 2 / 6894.757)

# Specific gravities (relative to water at 1000 kg/m³)
SG_LOX    = RHO_LOX / 1000.0   # 1.090896
SG_IPA    = RHO_IPA / 1000.0   # 0.785093

# Injector CdA values from Simulink LOX_flow / IPA_flow MATLAB functions
CDA_LOX   = 2.58e-5   # m²  (from LOX_flow: CdA = 2.58E-05)
CDA_IPA   = 2.53e-5   # m²  (from IPA_flow: CdA = 2.53E-05)

# Cv-to-flow correlation constant k (from LOX_flow in tv_controller_2_0.slx)
K_CV      = 7.598e-7   # m² (orifice area equivalent per unit Cv)

# Effective exhaust velocity for thrust estimation (from Simulink Gain7/Gain8)
# thrust_lbf = mdot_total * V_E_MS * 0.22482  →  V_E_MS = 1752.461343 m/s
V_E_MS    = 1752.461343  # m/s
# c* closure constant: Pc[Pa] = K_PC * mdot_total[kg/s]
# Derived from gain_scheduling.m IC: K_PC = Pc_ic / mdot_tot_ic
#   = 2225793.818 Pa / 1.269423 kg/s = 1753390.10 Pa·s/kg
# Equivalent to c*_eff * c* / At with Dt=1.175 in throat.
K_PC      = 1753390.10   # Pa·s/kg

PSI2PA    = 6894.757   # Pa per psi
P_TANK_PSI = 500.0     # Propellant tank pressure (psi) — 500 psi nominal supply

# =============================================================================
# PHYSICS SIMULATION CLASS
# =============================================================================

class Physics:
    """
    Thread-safe real-time physics simulation of the Moonshine 2 propellant system.

    State variables:
      lox_deg, ipa_deg   — Current valve angles (set by mock_can_bridge via HTTP)
      pom_psi            — LOX manifold pressure (what POM sensor reads)
      pfm_psi            — IPA manifold pressure (what PFM sensor reads)
      pc_psi             — Chamber pressure (what PC sensor reads)
      mdot_lox, mdot_ipa — Mass flow rates (kg/s) for diagnostics
      thrust_lbf         — Estimated thrust in lbf for diagnostics
      mr                 — Oxidizer-to-fuel mixture ratio (mdot_lox / mdot_ipa)

    All access through step(), set_valves(), and snap() is protected by a lock
    to prevent data races between the physics thread and the HTTP server thread.
    """

    def __init__(self):
        self.lock      = threading.Lock()
        # Initial valve angles: exact THETA_O / THETA_F from gain_scheduling.m at 500 lbf
        self.lox_deg   = 41.06017851
        self.ipa_deg   = 40.67697413
        # IC pressures from gain_scheduling.m at 500 lbf operating point,
        # verified self-consistent with the Simulink LOX_flow physics in step().
        # Stable from tick 1 — no startup transient.
        self.pc_psi    = 322.8241   # 2225793.818 Pa — gain_scheduling.m
        self.pom_psi   = 370.7493   # 2556432.5   Pa — LOX manifold at IC
        self.pfm_psi   = 370.8182   # 2556907.5   Pa — IPA manifold at IC
        # Mass flows pre-seeded at IC values
        self.mdot_lox  = 0.692724   # kg/s at IC
        self.mdot_ipa  = 0.576699   # kg/s at IC
        self.thrust_lbf = 500.14    # lbf at IC
        self.mr        = 1.2012     # O/F at IC

    def step(self):
        """
        Advance the physics simulation by one timestep.

        Uses the exact LOX_flow / IPA_flow equations from tv_controller_2_0.slx:

          p_m = (K_CV² * rho * Cv² * p_tank/SG  +  2*CdA²*p_c)
                / (2*CdA²  +  K_CV²*rho*Cv²/SG)

          mdot = CdA * sqrt(2 * rho * max(p_m - p_c, 0))

        Pc is closed via the c* relation (matched to gain_scheduling.m):
          Pc = K_PC * mdot_total
        where K_PC = c*_eff * c* / At = 1753390.10 Pa·s/kg.
        This is equivalent to Pc = mdot_tot * c*_eff * c* / At and gives
        PC=322.82 psi, POM=370.78 psi, PFM=370.85 psi at the 500 lbf IC.
        Iteration is damped for stability from any starting seed.
        """
        # Read current valve state without holding lock during computation
        with self.lock:
            lox_deg = self.lox_deg
            ipa_deg = self.ipa_deg
            pc_pa   = self.pc_psi * PSI2PA if self.pc_psi > 0 else PSI2PA * 100.0

        cv_lox  = deg_to_cv(lox_deg)
        cv_ipa  = deg_to_cv(ipa_deg)
        p_tank  = P_TANK_PSI * PSI2PA   # Pa

        # LOX_flow numerator/denominator coefficients (from tv_controller_2_0.slx)
        # p_m = (A + 2*CdA²*pc) / (2*CdA² + B)
        A_lox = K_CV**2 * RHO_LOX * cv_lox**2 * p_tank / SG_LOX
        B_lox = K_CV**2 * RHO_LOX * cv_lox**2 / SG_LOX
        A_ipa = K_CV**2 * RHO_IPA * cv_ipa**2 * p_tank / SG_IPA
        B_ipa = K_CV**2 * RHO_IPA * cv_ipa**2 / SG_IPA

        # Fixed-point iteration: Pc = K_PC * mdot_total (c* relation), damped.
        # Converges in < 10 iterations from any reasonable seed.
        for _ in range(20):
            pom_pa = (A_lox + 2 * CDA_LOX**2 * pc_pa) / (2 * CDA_LOX**2 + B_lox)
            pfm_pa = (A_ipa + 2 * CDA_IPA**2 * pc_pa) / (2 * CDA_IPA**2 + B_ipa)
            dp_lox  = max(0.0, pom_pa - pc_pa)
            dp_ipa  = max(0.0, pfm_pa - pc_pa)
            mdot_lox = CDA_LOX * math.sqrt(2.0 * RHO_LOX * dp_lox) if dp_lox > 0 else 0.0
            mdot_ipa = CDA_IPA * math.sqrt(2.0 * RHO_IPA * dp_ipa) if dp_ipa > 0 else 0.0
            # c* closure: Pc[Pa] = K_PC * mdot_total  (K_PC = c*_eff*c*/At)
            pc_new = K_PC * (mdot_lox + mdot_ipa) if (mdot_lox + mdot_ipa) > 0 else 0.0
            pc_blended = 0.5 * pc_pa + 0.5 * pc_new
            if abs(pc_blended - pc_pa) / (pc_pa + 1.0) < 1e-6:
                pc_pa = pc_blended
                break
            pc_pa = pc_blended

        # Final pass at converged Pc
        pom_pa   = (A_lox + 2 * CDA_LOX**2 * pc_pa) / (2 * CDA_LOX**2 + B_lox)
        pfm_pa   = (A_ipa + 2 * CDA_IPA**2 * pc_pa) / (2 * CDA_IPA**2 + B_ipa)
        dp_lox   = max(0.0, pom_pa - pc_pa)
        dp_ipa   = max(0.0, pfm_pa - pc_pa)
        mdot_lox = CDA_LOX * math.sqrt(2.0 * RHO_LOX * dp_lox) if dp_lox > 0 else 0.0
        mdot_ipa = CDA_IPA * math.sqrt(2.0 * RHO_IPA * dp_ipa) if dp_ipa > 0 else 0.0
        mdot_tot = mdot_lox + mdot_ipa

        # Thrust matches Simulink Gain7+Gain8: thrust_lbf = mdot_tot * V_E_MS * 0.22482
        thrust_lbf = mdot_tot * V_E_MS * 0.22482014388489208
        mr = mdot_lox / mdot_ipa if mdot_ipa > 1e-6 else 0.0

        with self.lock:
            self.pom_psi    = pom_pa  / PSI2PA
            self.pfm_psi    = pfm_pa  / PSI2PA
            self.pc_psi     = pc_pa   / PSI2PA
            self.mdot_lox   = mdot_lox
            self.mdot_ipa   = mdot_ipa
            self.thrust_lbf = thrust_lbf
            self.mr         = mr

    def set_valves(self, lox_deg: float, ipa_deg: float):
        """
        Update simulated valve angles.  Called by mock_can_bridge.py at 10 Hz
        after it has advanced the valve's simulated slew toward the commanded angle.
        Clamps to [0, 90] degrees — the physical valve range.
        """
        with self.lock:
            self.lox_deg = max(0.0, min(90.0, float(lox_deg)))
            self.ipa_deg = max(0.0, min(90.0, float(ipa_deg)))

    def snap(self) -> dict:
        """Return a thread-safe snapshot of all current state as a dict."""
        with self.lock:
            return dict(
                pom_psi    = self.pom_psi,
                pfm_psi    = self.pfm_psi,
                pc_psi     = self.pc_psi,
                lox_deg    = self.lox_deg,
                ipa_deg    = self.ipa_deg,
                mdot_lox   = self.mdot_lox,
                mdot_ipa   = self.mdot_ipa,
                thrust_lbf = self.thrust_lbf,
                mr         = self.mr,
            )

# Create singleton physics instance — pressures are pre-set to IC values in
# __init__() so the physics is at steady-state from the first sensor read.
physics = Physics()

def physics_loop():
    """Run physics at ~170 Hz to match the Simulink controller sample rate."""
    while True:
        physics.step()
        time.sleep(1.0 / 170.0)   # 5.88 ms per step

threading.Thread(target=physics_loop, daemon=True).start()

# =============================================================================
# SENSOR ID MAPPING
# =============================================================================
# These must EXACTLY match the SENSOR_ID_* #defines in tv_main.c.
# The % in sensor IDs is URL-encoded as %23 (# = hash/pound sign).
# Format: b<board>_log_data_<type>#<channel>
#   board = 1 (Board 1)
#   type  = ads1256 (ADS1256 24-bit ADC chip)
#   channel = 0, 1, 2, ... (ADC input channel number)

SENSOR_ID_MAP = {
    "b1_log_data_ads1256%231": "pom_psi",   # Channel 1: LOX manifold pressure (POM)
    "b1_log_data_ads1256%232": "pfm_psi",   # Channel 2: IPA manifold pressure (PFM)
    "b1_log_data_ads1256%233": "pc_psi",    # Channel 3: Chamber pressure (PC)
}

SENSOR_TITLE_MAP = {
    "b1_log_data_ads1256%231": "POM",
    "b1_log_data_ads1256%232": "PFM",
    "b1_log_data_ads1256%233": "PC",
}

# =============================================================================
# HTTP REQUEST HANDLER
# =============================================================================

class Handler(BaseHTTPRequestHandler):
    """Handles DAQstra-compatible HTTP requests."""

    def handle_error(self):
        """Suppress BrokenPipe tracebacks — these are normal at 170 Hz read rate."""
        pass

    def log_message(self, *args):
        pass  # Suppress per-request console logs to reduce noise

    def send_json(self, code: int, obj: dict):
        """Send an HTTP response with JSON body.
        Silently ignores BrokenPipe / ConnectionReset — tv_main opens a new
        connection for every sensor read at 170 Hz and closes it very quickly,
        so the server-side write occasionally races the client disconnect.
        The data was already read successfully by the client before the pipe broke.
        """
        body = json.dumps(obj).encode()
        try:
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass  # Client closed connection before we finished — normal at 170 Hz

    def do_GET(self):
        path = urlparse(self.path).path.rstrip("/")

        # --- Health check (mirrors real DAQstra /health response) ---
        if path == "/health":
            self.send_json(200, {
                "status": "ok",
                "service": "mock_daq",
                "mqtt_connected": True,
                "sensors_loaded": len(SENSOR_ID_MAP),
            })
            return

        # --- List all sensors ---
        if path == "/api/v1/sensors":
            s = physics.snap()
            sensors = []
            for sid, key in SENSOR_ID_MAP.items():
                sensors.append({
                    "sensor_id": sid,
                    "title":     SENSOR_TITLE_MAP[sid],
                    "metrics":   {"latest_value": round(s[key], 3)},
                })
            self.send_json(200, {"sensors": sensors})
            return

        # --- Get sensor by ID (PRIMARY endpoint used by tv_main.c) ---
        # tv_main calls: GET /api/v1/sensors/b1_log_data_ads1256%230
        # urlparse may or may not decode the %23, so we handle both forms.
        sensor_prefix = "/api/v1/sensors/"
        if path.startswith(sensor_prefix):
            raw_id      = path[len(sensor_prefix):]
            sid_encoded = raw_id.replace("#", "%23")   # ensure encoded form
            s = physics.snap()
            if sid_encoded in SENSOR_ID_MAP:
                key = SENSOR_ID_MAP[sid_encoded]
                self.send_json(200, {
                    "sensor_id": sid_encoded,
                    "title":     SENSOR_TITLE_MAP[sid_encoded],
                    "metrics":   {"latest_value": round(s[key], 3)},
                })
            elif raw_id in SENSOR_ID_MAP:
                # Accept un-encoded form as fallback
                key = SENSOR_ID_MAP[raw_id]
                self.send_json(200, {
                    "sensor_id": raw_id,
                    "title":     SENSOR_TITLE_MAP.get(raw_id, raw_id),
                    "metrics":   {"latest_value": round(s[key], 3)},
                })
            else:
                self.send_json(404, {"error": f"Unknown sensor_id '{raw_id}'"})
            return

        # --- Mock-only debug endpoint (not in real DAQstra) ---
        if path == "/mock/status":
            s = physics.snap()
            s["cv_lox"] = round(deg_to_cv(s["lox_deg"]), 4)
            s["cv_ipa"] = round(deg_to_cv(s["ipa_deg"]), 4)
            self.send_json(200, s)
            return

        self.send_json(404, {"error": "not found"})

    def do_POST(self):
        path = urlparse(self.path).path.rstrip("/")
        n    = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n)) if n else {}

        # --- Update valve angles (called by mock_can_bridge.py at 10 Hz) ---
        # body: {"lox_deg": 41.06, "ipa_deg": 40.68}
        if path == "/mock/valve_angles":
            physics.set_valves(
                body.get("lox_deg", 41.06),
                body.get("ipa_deg", 40.68)
            )
            self.send_json(200, {"ok": True})
            return

        self.send_json(404, {"error": "not found"})


# =============================================================================
# ENTRY POINT
# =============================================================================

# =============================================================================
# QUIET HTTP SERVER — suppresses BrokenPipe tracebacks at 170 Hz read rate
# =============================================================================

class QuietHTTPServer(HTTPServer):
    """HTTPServer subclass that silently drops BrokenPipe/ConnectionReset errors.

    tv_main opens a new HTTP connection for every sensor read (170 Hz x 3 sensors
    = 510 connections/sec) and closes them very quickly via the 30ms libcurl
    timeout.  The server-side write occasionally races the client disconnect,
    producing BrokenPipeError.  The data was already read successfully by the
    client before the pipe broke — these errors are harmless noise.

    Python 3.13 changed how socketserver propagates exceptions through
    handle_error, so we check both sys.exc_info() and BaseException context.
    """
    def handle_error(self, request, client_address):
        import sys, traceback
        # Try current exception context first (Python 3.13+)
        exc = sys.exc_info()[1]
        if exc is None:
            # Python 3.13 may store it differently — check __context__
            try:
                exc_type, exc, tb = sys.exc_info()
            except Exception:
                pass
        if isinstance(exc, (BrokenPipeError, ConnectionResetError, OSError)):
            # OSError errno 32 = Broken pipe, errno 104 = Connection reset
            return  # Normal at 170 Hz — suppress completely
        # Check by traceback string as fallback for Python 3.13
        tb_str = traceback.format_exc()
        if 'BrokenPipeError' in tb_str or 'ConnectionResetError' in tb_str:
            return
        super().handle_error(request, client_address)


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Josh Throttle Mock DAQ Server")
    ap.add_argument("--port", type=int, default=8050,
                    help="Port to listen on (default: 8050, matches real DAQstra)")
    args = ap.parse_args()

    print(f"[MockDAQ] Josh Throttle Mock DAQ Server")
    print(f"[MockDAQ] Listening on http://0.0.0.0:{args.port}")
    print(f"[MockDAQ] Physics: Moonshine 2 orifice flow model at 170 Hz")
    print(f"[MockDAQ] Sensor endpoints:")
    for sid, title in SENSOR_TITLE_MAP.items():
        print(f"  GET /api/v1/sensors/{sid}  \u2192 {title}")
    print(f"[MockDAQ] Set valve angles: POST /mock/valve_angles")
    print(f"[MockDAQ] Debug physics:    GET /mock/status")

    QuietHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()