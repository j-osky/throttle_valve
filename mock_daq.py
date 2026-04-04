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
The simulation implements the same equations used in the Simulink fluids model:

1. VALVE Cv: Each valve has a flow coefficient (Cv) that depends on its angle.
   This is a lookup table identical to the "deg → Cv" Simulink block.
   Cv represents how easily fluid flows through the valve — it increases
   nonlinearly with angle (it's roughly exponential near full open).

2. MASS FLOW (orifice equation):
   mdot = CdA * Cv * sqrt(2 * rho * dP)
   where:
     CdA   = discharge coefficient × orifice area (fixed for each propellant)
     rho   = propellant density (kg/m³)
     dP    = Ptank - Pc (pressure drop across the injector orifice)

3. CHAMBER PRESSURE convergence (iterated 8 times per step):
   Pc = mdot_total * cstar_eff * cstar / At
   where cstar = characteristic velocity and At = throat area.
   This is a fixed-point iteration because Pc depends on mdot and mdot
   depends on dP = Ptank - Pc.

4. INJECTOR MANIFOLD PRESSURES:
   Pom = Pc + (mdot_lox / CdA_lox)² / (2 * rho_lox)
   Pfm = Pc + (mdot_ipa / CdA_ipa)² / (2 * rho_ipa)
   These are the sensor readings that tv_main's PI controllers actually use.

DAQSTRA API COMPATIBILITY
--------------------------
tv_main.c calls these endpoints:
  GET /api/v1/sensors/<sensor_id>   → returns {"metrics": {"latest_value": N}}
  GET /api/v1/sensors               → lists all sensors
  GET /health                       → service health check

The sensor IDs must EXACTLY MATCH the SENSOR_ID_* defines in tv_main.c.
Default mapping:
  b1_log_data_ads1256%230  →  POM (LOX manifold pressure, psi)
  b1_log_data_ads1256%231  →  PFM (IPA manifold pressure, psi)
  b1_log_data_ads1256%232  →  PC  (chamber pressure, psi)

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
# PROPELLANT PHYSICAL CONSTANTS  (from MATLAB sizing script)
# =============================================================================

RHO_LOX   = 68.1 * 16.018463    # LOX density:  68.1 lb/ft³ → 1090.9 kg/m³
RHO_IPA   = 49.1 * 16.018463    # IPA density:  49.1 lb/ft³ →  786.5 kg/m³

# Injector discharge coefficient × orifice area for each propellant.
# These come from the MATLAB sizing script (CdA_lox_in2 and CdA_fuel_all_in2)
# converted from in² to m² (1 in² = 6.4516e-4 m²).
CDA_LOX   = 0.03993928131 * 0.00064516   # m² = 2.576e-5 m²
CDA_IPA   = 0.0392328813  * 0.00064516   # m² = 2.531e-5 m²

CSTAR_EFF = 0.70     # Combustion efficiency (c* efficiency) — from MATLAB (70%)
CSTAR     = 1752.0   # Characteristic velocity c* in m/s (from CEA at 500 lbf)
AT        = math.pi * (1.170 * 0.0254)**2 / 4  # Throat area m² (Dt=1.170 in)
PSI2PA    = 6894.757 # Conversion factor: 1 psi = 6894.757 Pa
P_TANK_PSI = 550.0   # Propellant tank pressure (psi) — approximate supply pressure

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
        # Initial valve angles match Simulink THETA_O and THETA_F at 500 lbf
        self.lox_deg   = 41.06
        self.ipa_deg   = 40.68
        # Computed pressures (psi) — updated by step()
        self.pom_psi   = 0.0
        self.pfm_psi   = 0.0
        self.pc_psi    = 0.0
        # Mass flows and derived quantities — for diagnostics only
        self.mdot_lox  = 0.0
        self.mdot_ipa  = 0.0
        self.thrust_lbf = 0.0
        self.mr        = 0.0

    def step(self):
        """
        Advance the physics simulation by one timestep.

        Algorithm:
        1. Read current valve angles (under lock, then release to avoid
           holding the lock during the computation).
        2. Convert angles to Cv using the lookup table.
        3. Iteratively solve for Pc using the cstar relation and orifice flow.
           This is necessary because mdot depends on dP = Ptank - Pc and Pc
           depends on mdot.  8 iterations gives excellent convergence (error < 0.01%).
        4. Compute manifold pressures Pom and Pfm from the converged mdots.
        5. Write all results back (under lock).
        """
        # Read current valve state without holding lock during computation
        with self.lock:
            lox_deg = self.lox_deg
            ipa_deg = self.ipa_deg
            pc_pa   = self.pc_psi * PSI2PA if self.pc_psi > 0 else 1e5  # initial guess

        cv_lox = deg_to_cv(lox_deg)
        cv_ipa = deg_to_cv(ipa_deg)
        p_tank = P_TANK_PSI * PSI2PA  # tank pressure in Pa

        # Fixed-point iteration: converge Pc (8 iterations)
        for _ in range(8):
            dp_lox = max(0.0, p_tank - pc_pa)   # pressure drop across LOX orifice
            dp_ipa = max(0.0, p_tank - pc_pa)   # pressure drop across IPA orifice

            # Incompressible orifice flow: mdot = CdA * Cv * sqrt(2 * rho * dP)
            mdot_lox = CDA_LOX * cv_lox * math.sqrt(2.0 * RHO_LOX * dp_lox) if dp_lox > 0 else 0.0
            mdot_ipa = CDA_IPA * cv_ipa * math.sqrt(2.0 * RHO_IPA * dp_ipa) if dp_ipa > 0 else 0.0

            # Chamber pressure from cstar relation: Pc = mdot_total * cstar / At
            mdot_tot = mdot_lox + mdot_ipa
            pc_new   = mdot_tot * CSTAR_EFF * CSTAR / AT

            # Check convergence
            if abs(pc_new - pc_pa) / (pc_pa + 1) < 1e-4:
                break
            pc_pa = pc_new

        # Manifold pressures: Pinj = Pc + (mdot/CdA)² / (2*rho)
        # This is the injector pressure drop equation solved for upstream pressure.
        pom_pa = pc_pa + (mdot_lox / CDA_LOX)**2 / (2.0 * RHO_LOX) if mdot_lox > 0 else pc_pa
        pfm_pa = pc_pa + (mdot_ipa / CDA_IPA)**2 / (2.0 * RHO_IPA) if mdot_ipa > 0 else pc_pa

        # Thrust: F = mdot_total * ve (simplified — cstar used as ve proxy)
        thrust_lbf = mdot_tot * CSTAR_EFF * CSTAR / 4.4482216  # N → lbf

        # Mixture ratio: O/F = mdot_lox / mdot_ipa
        mr = mdot_lox / mdot_ipa if mdot_ipa > 1e-6 else 0.0

        # Write results back under lock
        with self.lock:
            self.pom_psi    = pom_pa / PSI2PA
            self.pfm_psi    = pfm_pa / PSI2PA
            self.pc_psi     = pc_pa  / PSI2PA
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

# Create singleton physics instance and start the simulation thread
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
    "b1_log_data_ads1256%230": "pom_psi",   # Channel 0: LOX manifold pressure (POM)
    "b1_log_data_ads1256%231": "pfm_psi",   # Channel 1: IPA manifold pressure (PFM)
    "b1_log_data_ads1256%232": "pc_psi",    # Channel 2: Chamber pressure (PC)
}

SENSOR_TITLE_MAP = {
    "b1_log_data_ads1256%230": "POM",
    "b1_log_data_ads1256%231": "PFM",
    "b1_log_data_ads1256%232": "PC",
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