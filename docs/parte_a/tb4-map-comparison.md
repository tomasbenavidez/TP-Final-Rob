# Comparación de mapas TB4 con odometría bracketed

Fecha: 2026-06-30.

## Entrada y procedimiento

- Bag: `tp_final_ws/bags/laberinto` (1392.763 s, 10 797 scans).
- Reproducción: `ros2 bag play ... --clock --rate 4.0
  --disable-keyboard-controls`.
- Primera pasada: 287 poses optimizadas y 42 landmarks.
- Segunda pasada A: `max_angular_velocity:=0.0`.
- Segunda pasada B: `max_angular_velocity:=0.35`.
- Resolución: 0.05 m/celda.
- Extrínsecos: TF del bag; fallback `tx=-0.04`, `yaw=pi/2`.

Los artefactos completos y logs quedaron fuera del checkout en
`/tmp/tb4-bracketing-diagnostic`.

## Resultados

| Métrica | Sin gate angular | Gate 0.35 rad/s |
|---|---:|---:|
| Scans recibidos con bracket | 10 770 | 10 784 |
| Scans integrados | 10 770 | 10 783 |
| Descartados por giro | 0 | 1 |
| Descartados por espera | 1 | 1 |
| Descartados al finalizar | 0 | 0 |
| TF real / fallback | 10 769 / 1 | 10 782 / 1 |
| Máximo gap odométrico | 946.9 ms | 590.1 ms |
| Celdas ocupadas | 2 058 | 2 058 |
| Celdas libres | 9 115 | 9 116 |
| Celdas distintas entre mapas | 1 | 1 |
| Espesor ocupado mediano | 0.10 m | 0.10 m |
| Espesor ocupado p95 | 0.28 m | 0.28 m |
| Componentes ocupados pequeños (≤20 celdas) | 34 | 34 |

La reproducción acelerada produjo una diferencia de recepción entre corridas:
26 y 12 scans del bag, respectivamente, no llegaron al callback. Por eso los
conteos de transporte no son bit a bit deterministas. Dentro de cada corrida,
ningún scan fue integrado por extrapolación y la cola terminó vacía.

## Inspección

- Las aberturas principales permanecen abiertas en ambas variantes.
- No se observan conexiones falsas nuevas ni paredes dobles severas.
- Los 34 componentes pequeños se conservan en ambas variantes, por lo que el
  gate angular no aporta una mejora medible de objetos finos en este bag.
- Ambos mapas tienen 42 componentes ocupados y difieren en una única celda
  libre/desconocida; el único scan descartado por el gate no altera paredes.

## Decisión

Se conserva `max_angular_velocity:=0.0` como default. El bracketing elimina el
clamp al último odom sin necesitar cambios de log-odds. El gate de 0.35 rad/s
queda disponible para diagnósticos, pero este bag no aporta evidencia para
activarlo por defecto.

Para una comparación bit a bit de transporte, repetir a `--rate 1.0` o con un
reproductor offline; la evidencia actual sí valida el contrato de bracketing,
TF/fallback, cierre de cola y equivalencia geométrica de las dos variantes.

## Gates A1–A3 del bag conocido

| Gate | Evidencia local | Estado |
|---|---|---|
| A1 | 42 landmarks; TF real en 10 769/10 770 y 10 782/10 783 integraciones; un fallback documentado por corrida | Verificado para este bag |
| A2 | 287 poses optimizadas; trayectoria y landmarks exportados | Datos verificados; inspección/captura de suavidad pendiente |
| A3 | Mapas cargables, aberturas principales abiertas, sin conexiones falsas nuevas ni paredes dobles severas | Verificado para este bag |

Los artefactos “sin gate angular” y “gate 0.35 rad/s” constituyen,
respectivamente, la comparación baseline y bracketed registrada. Esta
validación no aprueba un mapa adquirido en otra fecha ni reemplaza la
inspección física del escenario y del `RUN_ID`.
