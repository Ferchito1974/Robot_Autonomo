# AMR U3: control de velocidad de rueda y trayectorias omni

**Advanced Mobile Robotics, Assessment 1 Unit 3** (Universidad Politécnica de Yucatán).
Control de velocidad de las 3 ruedas de un robot omnidireccional (FutBot Omni) con un controlador
**P** y un **PID** (reto extra), y ejecución de trayectorias **recta, círculo y cuadrado** a partir
de referencias de velocidad, con odometría por encoders y giroscopio.

**Informe:** [`reporte/AMR_ASS1_U3_Fernando_Euan.pdf`](reporte/AMR_ASS1_U3_Fernando_Euan.pdf)
(se regenera con `../tarea_odometria/venv/bin/python reporte/build_report.py` desde esta carpeta).

---

## Qué hace

| Parte | Dónde corre | Descripción |
|---|---|---|
| Sensores y actuadores | Arduino Mega 2560 (firmware `FutBot_fw_v5`) | Lee encoders e IMU (BNO085), aplica PWM con `{PWM,p1,p2,p3}`, telemetría a 50 Hz, paro si no recibe comandos en 500 ms |
| Control, estimación y registro | Raspberry Pi 5 (Python) | Filtro de Kalman por rueda, cinemática omni, controlador P/PID por rueda a 50 Hz, odometría, registro CSV y gráficas |
| Interfaz | Navegador de la laptop | Elegir trayectoria, perfil de velocidad y controlador, ver gráficas y telemetría en vivo, calibrar |

```
 referencias (Vx, Vy) ──► rotación al marco del robot (giroscopio) ──► cinemática inversa omni
        ▲                         ▲  rumbo fijo en 0° (P de rumbo)            │ RPM de referencia ×3
        │                         │                                           ▼
  perfil trapezoidal        Kalman + giroscopio ◄── encoders ◄── robot ◄── PID por rueda + feedforward
```

### Controlador de rueda (`controller.py`)

```
e = RPM_ref − RPM_medida            (medida = filtro de Kalman sobre las cuentas del encoder)
u = FF(RPM_ref futura) + Kp·e + I − Kd·dRPM/dt          [PWM, saturado en ±255]
I += Ki·e·dt   (anti-windup: se congela si satura, |I| ≤ I_max)
FF = signo · (offset + |RPM| / K)   modelo estático del motor, calibrado en el piso
```

- **P**: Kp = 0.4 (Ki = Kd = 0). **PID**: Kp = 0.3, Ki = 4, Kd = 0 (Ti = Kp/Ki = 75 ms).
- **Anticipación del feedforward** (0.10 s): el FF usa la referencia futura para compensar el retardo
  de actuación medido (~40 ms) y la constante del motor (30 ms).
- **Empujón de arranque**: una rueda parada con referencia recibe al menos 35 PWM (fricción estática).

### Movimiento omni

Al iniciar, el giroscopio fija la orientación como **0°**. Las trayectorias se definen como
velocidades en el mundo (Vx, Vy); en cada ciclo se rotan al marco del robot con el ángulo medido y
se reparten entre las 3 ruedas. Un PI de rumbo (Kp 4, Ki 2) mantiene la orientación en 0°: el robot **no gira**,
se traslada en cualquier dirección. El cuadrado son 4 lados en +X, +Y, −X, −Y sin vueltas.

Cinemática (ruedas a 270°, 30° y 150°, L = 95 mm, +vy al frente):

```
v_i = −sin(a_i)·vx + cos(a_i)·vy + L·ω
```

Es la versión de 3 ruedas de la regla diferencial de la tarea, v_R,L = v ± ω·b/2.

---

## Cómo correrlo

Desde la Pi (o por SSH):

```bash
cd tarea_velocidad_u3
./start.sh          # robot real (/dev/ttyACM0)
./start.sh demo     # robot virtual, sin hardware (respaldo para la exposición)
./stop.sh
```

Luego abre en la laptop `http://<ip-de-la-pi>:5002`.

Requisitos: Python 3.11 con `flask`, `numpy`, `matplotlib`, `pyserial`
(se usa el venv de `tarea_odometria/`). Firmware `FutBot_fw_v5` en el Arduino; no hay que flashear nada nuevo.

### Orden recomendado en un piso nuevo

1. **Calibrar en el piso**: el robot gira en su lugar a 80 y 160 RPM y ajusta el feedforward de las 3 ruedas.
2. **Odometría → Distancia (k_lin)**: corre una recta, mide con cinta lo que avanzó y escríbelo.
3. **Odometría → Giro (k_rot)**: 2 vueltas en su lugar; el factor sale del giroscopio.
4. Elegir **perfil de velocidad** y correr recta, círculo y cuadrado con **P** y con **PID**.

### Perfiles de velocidad

| Perfil | v | Aceleración | Uso |
|---|---|---|---|
| 1 Base | 0.50 m/s | 1.2 m/s² | Pisos que derrapan |
| 2 Medio | 0.65 m/s | 1.5 m/s² | |
| 3 Rápido | 0.75 m/s | 1.8 m/s² | |
| 4 Muy rápido | 0.85 m/s | 2.0 m/s² | Piso liso (a 0.95 m/s las ruedas saturaban 30 % del tiempo) |

Las rampas son **en S** (jerk limitado): la aceleración sube gradualmente, lo que reduce el derrape
al arrancar. Con rampas en S, el cuadrado omni a 0.75 m/s pasó de girar 28 a 44° por derrape a 16 a 24°,
con error final de 0.2 a 0.7 cm.

**Control de tracción** (opcional, desactivado por defecto, en "Ajustes avanzados"): el derrape se mide como la diferencia entre
la velocidad de giro de las ruedas y la del giroscopio. Si pasa de 1 rad/s, la referencia avanza más
lento en el tiempo (hasta 40 %) y se recupera poco a poco; la forma de la trayectoria no cambia.
Los cambios de escala son lentos (0.7 y 0.5 por segundo): en simulación, cambios rápidos frenaban tan
fuerte que causaban más derrape. En el robot real (piso liso, 0.75 m/s, 1.8 m/s²) no dio una mejora
medible: con tracción el giro máximo fue 7 a 17° en el cuadrado y 8 a 11° en el círculo; sin tracción,
6 a 10° y 6 a 9°. Lo que sí redujo el derrape fue bajar la aceleración, las rampas en S y limitar la
corrección de rumbo a 2 rad/s. En el robot real, con aceleraciones de 2.2 a 2.5 m/s² el robot giraba 28 a 44° por
derrape al iniciar cada lado del cuadrado; con 1.5 a 2.0 m/s², 5 a 17°.

---

## Seguridad

- Botón **STOP** y paro automático del firmware si no llegan comandos en 500 ms.
- Se aborta la corrida si: se pierde la telemetría (300 ms), hay un salto de encoder imposible,
  una rueda gira **contra su PWM** 0.15 s (falla de encoder) o una rueda queda **saturada** 1 s.
- Reposo de 1.5 s antes de cada corrida para medir el bias del giroscopio.

---

## Resultados (robot real, piso liso, 25-nov-2026)

| Corrida | Controlador | Error final | Rumbo final | Error de rueda RMS |
|---|---|---|---|---|
| Cuadrado omni 0.6 m, 0.5 m/s | P (Kp 0.4) | 2.5 a 2.8 cm | −1° | ~33 RPM |
| Cuadrado omni 0.6 m, 0.5 m/s | PID (Kp 0.3, Ki 4) | 0.6 a 1.0 cm | 0 a 1° | 19 a 22 RPM |
| Círculo omni R 0.35 m, 0.5 m/s | P | 1.8 cm | 0° | ~30 RPM |
| Círculo omni R 0.35 m, 0.5 m/s | PID | 0.8 a 1.0 cm | 0 a 2° | 13 a 16 RPM |

Las posiciones son la estimación de odometría (encoders + giroscopio) con k_lin = 1.06 calibrado con cinta.

**Efecto del feedforward** (cuadrado omni, PID Kp 0.3 Ki 4): con FF el error de rueda es de 24 a 45 RPM
y la desviación de trayectoria de 0.7 a 2 cm; sin FF, 61 a 89 RPM y 4 a 7 cm (la rueda va ~140 ms
atrasada porque el PID tiene que generar todo el PWM a partir del error).

---

## Archivos

| Archivo | Contenido |
|---|---|
| `app.py`, `ui.html` | Servidor Flask (puerto 5002) e interfaz web |
| `experiment.py` | Una corrida: reposo, referencias, control a 50 Hz, seguridad, guardado; calibración en el piso |
| `controller.py` | Controlador de rueda P/PID con feedforward, anti-windup y empujón de arranque |
| `trajectories.py` | Referencias: perfiles trapezoidales exactos, modo diferencial y modo omni |
| `kinematics.py` | Cinemática directa e inversa de la base omni (con k_lin y k_rot) |
| `estimator.py` | Kalman por rueda, odometría y rumbo (giroscopio o encoders) |
| `link.py`, `fake_link.py` | Enlace serial con el Arduino y robot virtual (incluye el retardo real de 40 ms) |
| `config.py`, `settings.py` | Constantes y ajustes editables (`settings.json`) |
| `plots.py` | Gráficas de cada corrida |
| `runs/` | Cada corrida: `data.csv`, `summary.json`, `wheels.png`, `velocity.png`, `xy.png`, `orientation.png` |
