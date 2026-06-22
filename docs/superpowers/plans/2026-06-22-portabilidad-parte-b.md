# Portabilidad de Parte B Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminar rutas personales de la rama Parte B y documentar un flujo portable Parte A → mapa → Parte B.

**Architecture:** Los mapas fuente permanecen en `mapas/` y setuptools los instala en el share de `tp_b_navigation`. Los ejecutables resuelven recursos mediante el índice de paquetes ROS; scripts y documentación usan su ubicación o rutas relativas.

**Tech Stack:** ROS 2 Humble, ament Python, setuptools, Python pathlib, zsh, pytest.

---

### Task 1: Contratos de portabilidad

**Files:**
- Create: `tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py`
- Modify: `tp_final_ws/src/tp_b_navigation/setup.py`
- Modify: `tp_final_ws/src/tp_b_navigation/package.xml`
- Modify: `tp_final_ws/src/tp_b_navigation/launch/parte_b.launch.py`
- Modify: `tp_final_ws/src/tp_b_navigation/launch/parte_b_localization.launch.py`
- Modify: `tp_final_ws/src/tp_b_navigation/tp_b_navigation/map_loader.py`

- [ ] Escribir tests que rechacen `/Users/` y `Documents/GitHub`, comprueben `share/tp_b_navigation/maps`, y exijan `get_package_share_directory` en los defaults.
- [ ] Ejecutar `python3 -m pytest tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py -q` y confirmar fallos por las rutas actuales.
- [ ] Agregar los mapas a `data_files`, declarar `ament_index_python`, y resolver `maps/map.yaml` desde el share del paquete.
- [ ] Repetir el test hasta obtener todos los casos en verde.

### Task 2: Scripts y documentación portable

**Files:**
- Modify: `docs/parte_b/scripts/setup_parte_b.sh`
- Modify: `docs/parte_b/scripts/gen_landmarks.py`
- Modify: `docs/parte_b/01_implementacion.md`
- Modify: `docs/parte_b/02_guia_ejecucion.md`
- Modify: `README.md`
- Move: `AGENT.md` → `AGENTS.md`

- [ ] Extender el test de portabilidad para cubrir scripts, README, guías y existencia exclusiva de `AGENTS.md`; confirmar el fallo.
- [ ] Hacer que los scripts deriven `REPO_ROOT` desde su propio archivo y permitan overrides para dependencias externas.
- [ ] Reescribir README con build y ejecución relativos para ambas partes.
- [ ] Renombrar y actualizar `AGENTS.md` con arquitectura, límites y comandos portables.
- [ ] Corregir las guías de Parte B y ejecutar nuevamente el test.

### Task 3: Regresión y cierre

**Files:**
- Test: `tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py`
- Test: `tp_final_ws/src/tp_slam_aruco/test/`

- [ ] Ejecutar `python3 -m compileall -q tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_slam_aruco`.
- [ ] Ejecutar tests que no requieran runtime ROS y registrar cualquier dependencia ausente.
- [ ] Ejecutar `git diff --check` y un escaneo final de rutas personales.
- [ ] Revisar el diff para confirmar que no cambiaron algoritmos, tópicos, frames ni parámetros numéricos.
