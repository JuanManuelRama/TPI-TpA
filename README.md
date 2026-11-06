# Simulador PID en Python

La interfaz visual se conserva a partir del HTML original. El cálculo del PID y
la planta se ejecutan en Python.

## Requisitos

- Python 3.9 o superior
- Flask
- NumPy

## Instalación y ejecución

En una terminal, dentro de esta carpeta:

```bash
python -m venv .venv
```

Activá el entorno virtual:

**Windows (PowerShell):**

```powershell
.venv\\Scripts\\Activate.ps1
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

Abrí en el navegador:

http://127.0.0.1:5000

## Método numérico

La planta es una ganancia constante:

```text
G(s) = K
```

Por lo tanto, en cada muestra la salida es directamente proporcional a la
entrada de la planta:

```text
theta[k] = K * ue[k]
```

No hay dinámica ni estado interno de la planta. El paso de simulación es
`dt = 0.02 s`.

El controlador es PID paralelo PI-D:

```text
u(t) = Kp*e(t) + Ki*integral(e) - Kd*d(theta)/dt
```

La derivada se aplica sobre la medición (no sobre el error), para evitar el
impulso derivativo cuando cambia el setpoint. La integral se aproxima mediante
la regla trapezoidal y se limita para reducir windup.

## Notas

- No incluye el modo de simulación en Laplace de la versión intermedia: parte
  del simulador HTML original.
- La interfaz conserva los mismos controles, paneles y gráfica.
- El servidor está pensado para uso local con una sesión de simulación.
