# Memoria técnica (paper IEEE) — TP Final Robótica

Informe de ingeniería en LaTeX (formato `IEEEtran`, conferencia) que documenta las
decisiones de diseño de las Partes A, B y C.

## Estructura
- `main.tex` — preámbulo, título, abstract, introducción, marco teórico y `\input` de las secciones.
- `sec_parte_a.tex` — Graph SLAM con ArUco (GTSAM/LM, cierre de lazo, dos pasadas, grilla).
- `sec_parte_b.tex` — navegación autónoma (MCL, sensor virtual, A*, pure-pursuit) + **diagrama de bloques de la FSM**.
- `sec_parte_c.tex` — exploración informativa + detección de conos + **diagrama de bloques de la FSM de misión**.
- `sec_conclusion.tex` — discusión, brecha sim-to-real, conclusión y trabajo futuro.
- `img/` — figuras: mapas SLAM, trayectoria+landmarks (generada), y capturas de Parte B/C.
- `main.pdf` — PDF compilado localmente (9 páginas) como referencia; el definitivo se genera en Overleaf.

## Compilar
En **Overleaf**: subir toda la carpeta `paper/` y compilar `main.tex` (2 pasadas). Overleaf ya trae `IEEEtran`, `babel-spanish` y `tikz`.

Local: requiere `texlive-publishers` (IEEEtran), `texlive-lang-spanish` (babel) y `texlive-pictures` (tikz).

## Diagramas de máquina de estados
Ambos están hechos en **TikZ nativo** (Fig. de Parte B y Parte C), autocontenidos, sin imágenes externas — se editan directo en el `.tex`.

## Pendiente de completar (robot real)
La Parte C marca explícitamente como *a completar* los resultados sobre hardware
físico (gates B1–B3 y misión de conos). Tras la ventana de laboratorio, incorporar:
- métricas de localización/navegación en el TB4 real,
- resultado de la misión de conos,
- análisis cuantitativo de la brecha sim-to-real (§ Discusión).
