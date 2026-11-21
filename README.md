# Simulador PID en Python

Simulador de un lazo de control cerrado con controlador PID. La simulación corre
de forma continua en el backend (Python) y empuja al navegador, por WebSocket,
solamente las muestras nuevas. El navegador no calcula nada: dibuja lo que recibe
y envía comandos (play/pausa, parámetros, perturbación).

## Requisitos

- Python 3.9 o superior
- Flask, flask-sock, python-control y NumPy (ver `requirements.txt`)

## Instalación y ejecución

En una terminal, dentro de esta carpeta:

```bash
python -m venv .venv
```

Activá el entorno virtual:

**Windows (PowerShell):**

```powershell
.venv\Scripts\Activate.ps1
```

**macOS / Linux:**

```bash
source .venv/bin/activate
```

Instalá las dependencias y ejecutá:

```bash
pip install -r requirements.txt
python app.py
```

Abrí en el navegador: <http://127.0.0.1:5000>

## Arquitectura

```text
 navegador (index.html)                 backend (app.py)
 ┌──────────────────────┐   ws://…/ws   ┌──────────────────────────────┐
 │ gráfico + variables  │ <──────────── │ hilo "sim-loop": 1 paso/20 ms │
 │ controles y formulario│ ────────────> │ en tiempo real, siempre vivo │
 └──────────────────────┘   comandos    └──────────────────────────────┘
```

- El hilo de simulación avanza un paso cada `DT = 0.02 s` con compensación de
  deriva (usa `time.perf_counter`). Si se atrasa, recupera hasta 5 pasos por vuelta
  y los manda juntos en un único mensaje.
- Cada cliente tiene su propia cola de salida: un navegador lento no frena la
  simulación (se le descartan muestras).
- Al conectarse, un cliente recibe el estado actual y los últimos 10 s de historial.
  Varias pestañas ven la misma simulación y se mantienen sincronizadas.

### Protocolo (JSON)

| Sentido | Mensaje | Descripción |
|---|---|---|
| cliente → servidor | `{"type":"run","on":bool}` | Play / pausa |
| cliente → servidor | `{"type":"dist","on":bool}` | Activa o desactiva la perturbación |
| cliente → servidor | `{"type":"reset"}` | Reinicia (pausa y apaga la perturbación) |
| cliente → servidor | `{"type":"params","params":{ref,Kp,Ki,Kd,Ka,Kpl,T,dist,ua_max}}` | Actualiza parámetros |
| servidor → cliente | `{"type":"samples","data":[…]}` | Solo las muestras nuevas |
| servidor → cliente | `{"type":"state",…}` | Parámetros, play/pausa, perturbación y, si hay que redibujar, el historial |
| servidor → cliente | `{"type":"error","message":…}` | Mensaje o parámetros inválidos |

Cambiar `ref`, `Kp`, `Ki`, `Kd`, `Ka`, `dist` o `ua_max` se aplica **en caliente**,
sin reiniciar. Cambiar `Kpl` o `T` reinicia la simulación, porque modifica el
estado interno de la planta.

## Modelo

Lazo cerrado con realimentación unitaria (`H = 1`):

```text
ref ──(+)──> PID ──> Actuador ──(+)──> Planta ──┬──> θo
      (-)                         ^             │
       └──────────────────────────┼─────────────┘
                                  d (perturbación)
```

| Bloque | Modelo |
|---|---|
| Error | `e = ref - θo` |
| Controlador | `u = Kp·e + Ki·∫e·dt − Kd·dθo/dt` |
| Actuador | `ua = clip(Ka·u, 0, ua_max)` |
| Carga efectiva | `ue = ua + d` |
| Planta | `Kpl / (T·s + 1)` |

Unidades usadas en la interfaz: latencia (`ref`, `θo`, `e`) en **ms**;
límite de concurrencia, perturbación y planta (`ua`, `ue`, `d`) en **solicitudes**;
`Kpl` en **ms/sol**. Las ganancias del controlador absorben la conversión.

## Método numérico

- **Planta**: se discretiza una sola vez con `control.c2d(..., method="zoh")`, que es
  exacta para una entrada constante durante cada paso `DT`. En cada paso:
  `x[k+1] = A·x[k] + B·ue[k]`, `θo[k+1] = C·x[k+1]`.
- **Integral**: regla trapezoidal, `I(k) = I(k−1) + (DT/2)·(e(k) + e(k−1))`.
- **Derivada**: sobre la **medición** (`−(θo[k] − θo[k−1]) / DT`), no sobre el error,
  para evitar el golpe derivativo al cambiar la referencia.
- **Saturación y anti-windup**: el actuador se limita a `[0, ua_max]`. Mientras está
  saturado el integrador no avanza (integración condicional). Así, tras un periodo
  saturado, la salida vuelve rápido a la referencia en lugar de quedar “cargada”.

## Notas

- El servidor está pensado para uso local y una única simulación compartida.
- Se ejecuta sin recarga automática (`debug=False`) para no duplicar el hilo de simulación.
- Si el navegador pierde la conexión, reintenta cada 1–5 s; el indicador de la
  cabecera muestra el estado.