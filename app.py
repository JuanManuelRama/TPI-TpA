from __future__ import annotations

from threading import Lock
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory
from scipy import signal
import numpy as np

BASE_DIR = Path(__file__).resolve().parent
app = Flask(__name__)
DT = 0.02  # 20 ms, igual que en la interfaz
_lock = Lock()

# Estado de una única sesión local del simulador.
_state = {
    "time": 0.0,
    "theta": 0.0,
    "integral": 0.0,
    "previous_error": 0.0,
    "previous_theta": 0.0,
    "plant_x": np.zeros(1, dtype=float),
    "plant_key": None,
}


def reset_state():
    global _state
    _state = {
        "time": 0.0,
        "theta": 0.0,
        "integral": 0.0,
        "previous_error": 0.0,
        "previous_theta": 0.0,
        "plant_x": np.zeros(1, dtype=float),
        "plant_key": None,
    }


def finite_number(data, key, default):
    value = float(data.get(key, default))
    if not np.isfinite(value):
        raise ValueError(f"El parámetro {key} debe ser un número finito.")
    return value


def plant_discretization(kpl: float, tau: float):
    """
    Discretiza G(s) = Kpl / (T*s + 1) con retención de orden cero.
    scipy.signal.cont2discrete devuelve (Ad, Bd, Cd, Dd, dt).
    """
    numerator = [kpl]
    denominator = [tau, 1.0]
    a, b, c, d = signal.tf2ss(numerator, denominator)
    return signal.cont2discrete((a, b, c, d), DT, method="zoh")


@app.get("/")
def index():
    return send_from_directory(BASE_DIR, "index.html")


@app.post("/api/reset")
def api_reset():
    with _lock:
        reset_state()
    return jsonify({"ok": True})


@app.post("/api/step")
def api_step():
    try:
        data = request.get_json(force=True)
        ref = finite_number(data, "ref", 10.0)
        dist_on = bool(data.get("distOn", False))
        dist_val = finite_number(data, "distVal", 5.0)
        kp = finite_number(data, "Kp", 1.0)
        ki = finite_number(data, "Ki", 0.1)
        kd = finite_number(data, "Kd", 0.01)
        ka = finite_number(data, "Ka", 1.0)
        kpl = finite_number(data, "Kpl", 1.0)
        tau = max(finite_number(data, "T", 1.0), 0.001)
        disturbance = dist_val if dist_on else 0.0

        with _lock:
            # Recalcular la representación discreta si cambian Kpl o T.
            plant_key = (kpl, tau)
            if _state["plant_key"] != plant_key:
                ad, bd, cd, dd, _ = plant_discretization(kpl, tau)
                _state["plant_x"] = np.zeros(ad.shape[0], dtype=float)
                _state["plant_key"] = plant_key
                _state["plant_ad"] = ad
                _state["plant_bd"] = bd
                _state["plant_cd"] = cd
                _state["plant_dd"] = dd

            theta = float(_state["theta"])
            error = ref - theta

            # PID paralelo PI-D: derivada sobre la medición para evitar
            # el "derivative kick" cuando cambia bruscamente el setpoint.
            # Integral trapezoidal: mejora la aproximación respecto de Euler
            # hacia adelante y evita que Ki se comporte como una ganancia P.
            integral_candidate = (
                _state["integral"]
                + ki * 0.5 * (error + _state["previous_error"]) * DT
            )
            integral_candidate = float(np.clip(integral_candidate, -500.0, 500.0))

            derivative_measurement = (theta - _state["previous_theta"]) / DT
            p_term = kp * error
            i_term = integral_candidate
            d_term = -kd * derivative_measurement
            control = p_term + i_term + d_term

            actuator_output = ka * control
            plant_input = actuator_output + disturbance

            # Planta discreta obtenida desde scipy.signal.cont2discrete.
            # Forma de estado: x[k+1]=Ad*x[k]+Bd*u[k], y[k]=Cd*x[k]+Dd*u[k].
            ad = _state["plant_ad"]
            bd = _state["plant_bd"]
            cd = _state["plant_cd"]
            dd = _state["plant_dd"]
            plant_x = _state["plant_x"]
            theta_next = float((cd @ plant_x + dd * plant_input).squeeze())
            plant_x_next = (ad @ plant_x + bd.flatten() * plant_input).astype(float)

            # Actualizar memoria para la próxima muestra.
            _state["plant_x"] = plant_x_next
            _state["theta"] = theta_next
            _state["integral"] = integral_candidate
            _state["previous_error"] = error
            _state["previous_theta"] = theta
            _state["time"] += DT

            return jsonify({
                "time": round(_state["time"], 8),
                "ref": ref,
                "e": error,
                "p": p_term,
                "i": i_term,
                "d_pid": d_term,
                "u": control,
                "ua": actuator_output,
                "ue": plant_input,
                "theta": theta_next,
                "d": disturbance,
            })

    except (ValueError, TypeError, KeyError) as exc:
        return jsonify({"error": str(exc)}), 400


if __name__ == "__main__":
    reset_state()
    print("Simulador PID disponible en http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=False)
