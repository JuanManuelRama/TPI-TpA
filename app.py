from threading import Lock
from flask import Flask, jsonify, request, send_file
import control as ct
import numpy as np


class Params:
    
    """Parameters for the PID controller."""
    
    def __init__(self, ref: float, Kp: float, Ki: float, Kd: float, Ka: float, Kpl: float, T: float, dist: float):
        self.ref = ref
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ka = Ka
        self.Kpl = Kpl
        self.Plant = ct.ss(ct.tf([Kpl], [T, 1.0]))
        self.dist = dist

    @staticmethod
    def default():
        """Return default parameters."""
        return Params(
            ref=10.0,
            Kp=1.0,
            Ki=0.5,
            Kd=0.01,
            Ka=1.0,
            Kpl=1.0,
            T = 1,
            dist = 5.0,
        )
    
class State:
    """State of the PID controller and continuous plant."""
    
    def __init__(self, time: float, theta: float, integral: float, previous_error: float, previous_theta: float, plant_x: np.ndarray, disturbance_on: bool):
        self.time = time
        self.theta = theta
        self.integral = integral
        self.previous_error = previous_error
        self.previous_theta = previous_theta
        self.plant_x = plant_x
        self.disturbance_on = disturbance_on
    
    @staticmethod
    def default(params: Params):
        """Return default state."""
        return State(
            time=0.0,
            theta=0.0,
            integral=0.0,
            previous_error=0.0,
            previous_theta=0.0,
            plant_x=np.zeros(params.Plant.nstates, dtype=float),
            disturbance_on=False,
        )

app = Flask(__name__)


_lock = Lock()

DT = 0.02  # Simulation step: 20 ms
_params = Params.default()
_state = State.default(_params)


def reset_simulation():
    """Reset all simulation variables."""

    global _state
    _state = State.default(_params)


def advance_simulation():
    """Advance the PID controller and continuous plant by one DT."""

    ref = _params.ref
    kp = _params.Kp
    ki = _params.Ki
    kd = _params.Kd
    ka = _params.Ka
    dist_value = _params.dist

    # Current plant output
    theta = _state.theta

    # Calculate error
    error = ref - theta


    # Integral term (trapezoidal integration formula)
    integral = _state.integral + 0.5 * (error + _state.previous_error) * DT

    # Derivative on measurement
    derivative = (error - _state.previous_error) / DT

    # PID output
    control_output = error * kp + integral * ki + derivative * kd

    # Actuator output
    actuator_output = ka * control_output

    # Disturbance
    disturbance = dist_value if _state.disturbance_on else 0.0

    plant_input = actuator_output + disturbance

    # Integrate the continuous-time plant over one time step.
    time_points = np.array([0.0, DT])

    inputs = np.array([[plant_input, plant_input]] ,dtype=np.float64)


    response = ct.forced_response(
        _params.Plant,
        time_points,
        inputs=inputs,
        initial_state=_state.plant_x,
        return_states=True
    )

    # Save the new plant state
    _state.plant_x = np.asarray(
        response.states[:, -1],
        dtype=float
    ).flatten()

    # New plant output
    theta_next = float(
        np.asarray(response.outputs).reshape(-1)[-1]
    )

    # Update simulation memory
    _state.theta = theta_next
    _state.integral = integral
    _state.previous_error = error
    _state.previous_theta = theta

    _state.time += DT

    # Return simulation data
    return {
        "time": _state.time,
        "ref": ref,
        "e": error,
        "p": error * kp,
        "i": integral * ki,
        "d_pid": derivative * kd,
        "u": control_output,
        "ua": actuator_output,
        "ue": plant_input,
        "theta": theta_next,
        "d": disturbance,
    }


@app.get("/")
def index():
    return send_file("index.html")


@app.post("/post/step")
def post_step():
    """Advance the simulation by one step."""

    with _lock:
        result = advance_simulation()

    return jsonify(result)


@app.post("/post/dist")
def post_dist():
    """Activate or deactivate the disturbance."""
    with _lock:
        _state.disturbance_on = not _state.disturbance_on
    return "", 204


@app.patch("/post/params")
def patch_params():
    """Update parameters and reset simulation."""

    global _params

    with _lock:
        _params = Params(**request.json)
        reset_simulation()

        return "", 204

@app.post("/post/reset")
def post_reset():
    """Reset the simulation and return its parameters."""

    with _lock:
        reset_simulation()

    return "", 204


if __name__ == "__main__":
    reset_simulation()

    print("PID simulator running at http://127.0.0.1:5000")

    app.run(
        host="127.0.0.1",
        port=5000,
        debug=False
    )
