"""Simulador PID de lazo cerrado.

Backend Flask + WebSocket. Un hilo de simulación corre de forma continua en
tiempo real (un paso cada DT) y empuja a los navegadores conectados solamente
las muestras nuevas. El navegador no pide nada: recibe y dibuja.

Protocolo (JSON sobre ws://host:5000/ws)

  cliente -> servidor
    {"type": "run",    "on": true|false}
    {"type": "dist",   "on": true|false}
    {"type": "reset"}
    {"type": "params", "params": {ref, Kp, Ki, Kd, Ka, Kpl, T, dist, ua_max}}

  servidor -> cliente
    {"type": "state",   "params": {...}, "running": bool, "dist_on": bool,
                        "dt": float, "history": [muestra, ...]   # history es opcional
    {"type": "samples", "data": [muestra, ...]}                  # solo lo nuevo
    {"type": "error",   "message": str}
"""

import json
import math
import queue
import threading
import time
from collections import deque
from pathlib import Path

import control as ct
import numpy as np
from flask import Flask, send_file
from flask_sock import Sock

DT = 0.02                 # paso de simulación [s]
HISTORY_S = 10.0          # historial que recibe un cliente nuevo [s]
MAX_CATCHUP_STEPS = 5     # pasos máximos por vuelta si el hilo se atrasa
LIMIT = 1e6               # cota de cualquier parámetro (evita desbordes)


class Params:
    """Parámetros del controlador, actuador, planta y perturbación."""

    FIELDS = ("ref", "Kp", "Ki", "Kd", "Ka", "Kpl", "T", "dist", "ua_max")

    def __init__(self, ref, Kp, Ki, Kd, Ka, Kpl, T, dist, ua_max):
        self.ref = ref
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ka = Ka
        self.Kpl = Kpl
        self.T = T
        self.dist = dist
        self.ua_max = ua_max
        # Planta Kpl/(T*s+1) discretizada una sola vez con retenedor de orden
        # cero (ZOH): es exacta para una entrada constante durante cada paso DT.
        self.Plant = ct.c2d(ct.ss(ct.tf([Kpl], [T, 1.0])), DT, method="zoh")

    @classmethod
    def default(cls):
        return cls(ref=10.0, Kp=1.0, Ki=0.5, Kd=0.01, Ka=1.0,
                   Kpl=1.0, T=1.0, dist=5.0, ua_max=100.0)

    @classmethod
    def from_dict(cls, d):
        """Valida lo que llega del navegador. Lanza ValueError si no sirve."""
        try:
            vals = {k: float(d[k]) for k in cls.FIELDS}
        except (KeyError, TypeError, ValueError):
            raise ValueError("Parámetros incompletos o no numéricos")
        if not all(math.isfinite(v) and abs(v) <= LIMIT for v in vals.values()):
            raise ValueError(f"Los parámetros deben ser finitos y |valor| <= {LIMIT:g}")
        vals["T"] = max(vals["T"], 0.001)
        vals["ref"] = max(vals["ref"], 0.0)      # una latencia negativa no existe
        if vals["ua_max"] <= 0:
            raise ValueError("ua_max debe ser mayor que 0")
        return cls(**vals)

    def to_dict(self):
        return {k: getattr(self, k) for k in self.FIELDS}


class State:
    """Memoria del lazo: planta, integrador y muestra anterior."""

    def __init__(self, k, theta, integral, previous_error, previous_theta,
                 plant_x, disturbance_on):
        self.k = k                              # número de paso (tiempo = k * DT)
        self.theta = theta
        self.integral = integral                # integral de e(t) SIN multiplicar por Ki
        self.previous_error = previous_error
        self.previous_theta = previous_theta
        self.plant_x = plant_x
        self.disturbance_on = disturbance_on

    @staticmethod
    def default(params, disturbance_on=False):
        return State(k=0, theta=0.0, integral=0.0, previous_error=0.0,
                     previous_theta=0.0,
                     plant_x=np.zeros(params.Plant.nstates, dtype=float),
                     disturbance_on=disturbance_on)


class Client:
    """Un navegador conectado.

    Tiene su propia cola de salida y un hilo que la vacía hacia el socket, así
    un cliente lento nunca frena la simulación.
    """

    def __init__(self, ws):
        self.ws = ws
        self.out = queue.Queue(maxsize=1000)

    def push(self, msg):
        try:
            self.out.put_nowait(json.dumps(msg, separators=(",", ":")))
        except queue.Full:
            pass                                # cliente lento: se descarta

    def pump(self):
        while True:
            data = self.out.get()
            if data is None:
                return
            try:
                self.ws.send(data)
            except Exception:
                return

    def close(self):
        self.out.put(None)


class Simulator:
    def __init__(self):
        self.lock = threading.Lock()
        self.params = Params.default()
        self.state = State.default(self.params)
        self.running = False
        self.history = deque(maxlen=int(HISTORY_S / DT))
        self.clients = set()

    # ------------------------------------------------------------------ lazo
    def _step_locked(self):
        """Avanza el lazo un paso DT. Debe llamarse con self.lock tomado."""
        p, s = self.params, self.state

        theta = s.theta                                   # salida medida (H = 1)
        error = p.ref - theta

        # Integral trapezoidal: I(k) = I(k-1) + (DT/2) * (e(k) + e(k-1))
        integral_new = s.integral + 0.5 * (error + s.previous_error) * DT

        # Derivada sobre la medición: evita el golpe derivativo al cambiar la
        # referencia (d(ref - theta)/dt = -dtheta/dt cuando ref es constante).
        derivative = -(theta - s.previous_theta) / DT

        p_term = p.Kp * error
        i_term = p.Ki * integral_new
        d_term = p.Kd * derivative
        u = p_term + i_term + d_term                      # acción de control

        # Actuador: ganancia + saturación del límite de concurrencia.
        ua_raw = p.Ka * u
        ua = min(max(ua_raw, 0.0), p.ua_max)
        saturated = ua != ua_raw

        # Anti-windup por integración condicional: si el actuador está
        # saturado, el integrador no avanza.
        if not saturated:
            s.integral = integral_new

        disturbance = p.dist if s.disturbance_on else 0.0
        ue = ua + disturbance                             # carga efectiva a la planta

        # Planta discreta: x[k+1] = A x[k] + B ue[k];  theta[k+1] = C x[k+1]
        P = p.Plant
        s.plant_x = P.A @ s.plant_x + (P.B * ue).reshape(-1)
        theta_next = float((P.C @ s.plant_x).reshape(-1)[0])

        s.previous_error = error
        s.previous_theta = theta
        s.theta = theta_next
        s.k += 1

        sample = {
            "time": s.k * DT, "ref": p.ref, "e": error,
            "p": p_term, "i": s.integral * p.Ki, "d_pid": d_term,
            "u": u, "ua": ua, "ue": ue, "theta": theta_next, "d": disturbance,
        }
        sample = {k: round(v, 5) for k, v in sample.items()}
        sample["sat"] = saturated
        return sample

    def run_forever(self):
        """Hilo de simulación: un paso cada DT, con compensación de deriva."""
        next_t = time.perf_counter()
        while True:
            with self.lock:
                running = self.running
            if not running:
                time.sleep(0.05)
                next_t = time.perf_counter()
                continue

            now = time.perf_counter()
            batch = []
            with self.lock:
                while next_t <= now and len(batch) < MAX_CATCHUP_STEPS:
                    batch.append(self._step_locked())
                    next_t += DT
                if next_t < now - 0.25:                   # muy atrasado: resincroniza
                    next_t = now
                if batch:
                    self.history.extend(batch)
                    self._broadcast_locked({"type": "samples", "data": batch})
            time.sleep(max(0.0, next_t - time.perf_counter()))

    # -------------------------------------------------------------- clientes
    def _broadcast_locked(self, msg):
        for c in self.clients:
            c.push(msg)

    def _state_msg_locked(self, with_history):
        msg = {"type": "state", "params": self.params.to_dict(),
               "running": self.running, "dist_on": self.state.disturbance_on,
               "dt": DT}
        if with_history:
            msg["history"] = list(self.history)
        return msg

    def add_client(self, client):
        with self.lock:
            self.clients.add(client)
            client.push(self._state_msg_locked(with_history=True))

    def remove_client(self, client):
        with self.lock:
            self.clients.discard(client)

    # --------------------------------------------------------------- comandos
    def handle(self, client, raw):
        try:
            msg = json.loads(raw)
            kind = msg.get("type")
            with self.lock:
                if kind == "run":
                    self.running = bool(msg["on"])
                    self._broadcast_locked(self._state_msg_locked(False))
                elif kind == "dist":
                    self.state.disturbance_on = bool(msg["on"])
                    self._broadcast_locked(self._state_msg_locked(False))
                elif kind == "reset":
                    self.state = State.default(self.params)
                    self.history.clear()
                    self.running = False
                    self._broadcast_locked(self._state_msg_locked(True))
                elif kind == "params":
                    new = Params.from_dict(msg["params"])
                    plant_changed = (new.Kpl, new.T) != (self.params.Kpl, self.params.T)
                    self.params = new
                    if plant_changed:
                        # El estado de la planta depende de Kpl y T: se reinicia.
                        self.state = State.default(new, self.state.disturbance_on)
                        self.history.clear()
                    # Ref, ganancias, perturbación y ua_max se aplican en caliente.
                    self._broadcast_locked(self._state_msg_locked(plant_changed))
                else:
                    raise ValueError(f"Mensaje desconocido: {kind!r}")
        except KeyError as exc:
            client.push({"type": "error", "message": f"Falta el campo {exc}"})
        except (ValueError, TypeError, AttributeError) as exc:
            client.push({"type": "error", "message": str(exc)})


app = Flask(__name__)
sock = Sock(app)
sim = Simulator()
INDEX = Path(__file__).with_name("index.html")


@app.get("/")
def index():
    return send_file(INDEX)


@sock.route("/ws")
def ws_endpoint(ws):
    client = Client(ws)
    threading.Thread(target=client.pump, daemon=True).start()
    sim.add_client(client)
    try:
        while True:
            sim.handle(client, ws.receive())      # lanza ConnectionClosed al cerrar
    finally:
        sim.remove_client(client)
        client.close()


if __name__ == "__main__":
    threading.Thread(target=sim.run_forever, daemon=True, name="sim-loop").start()
    print("Simulador PID en http://127.0.0.1:5000")
    app.run(host="127.0.0.1", port=5000, debug=False, threaded=True)