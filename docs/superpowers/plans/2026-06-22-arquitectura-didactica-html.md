# Didactic Architecture HTML Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current node-centric HTML with a self-contained panoramic map that teaches the complete TP through “Percibir → Estimar → Decidir → Actuar” and exposes exact ROS details on demand.

**Architecture:** Keep one semantic HTML document with embedded CSS, data, and JavaScript. The DOM contains one stable system map; a mode controller switches conceptual/ROS labels without replacing the geometry, a selection controller highlights connected blocks, and a tour controller walks four predefined paths. Part C is present only as an explicitly future, non-implemented region.

**Tech Stack:** HTML5, CSS custom properties/grid/media queries, vanilla JavaScript, Python `unittest` with `html.parser`, in-app browser for visual and interaction QA.

---

## File map

- Modify `tests/test_architecture_html.py`: structural regression tests for nodes, modes, tours, accessibility, critical contracts, and the future-only status of Part C.
- Modify `docs/arquitectura_sistema_completo.html`: the complete deliverable—semantic content, styles, system data, interaction state, and responsive layout.
- Do not modify ROS packages, launch files, maps, algorithms, or parameters.

### Task 1: Specify the new document contract with failing tests

**Files:**
- Modify: `tests/test_architecture_html.py`
- Test: `tests/test_architecture_html.py`

- [ ] **Step 1: Extend the parser with semantic state**

Add fields to `ArchitectureParser.__init__` and collect them in `handle_starttag`:

```python
self.modes = set()
self.tours = set()
self.capabilities = set()
self.future_sections = 0
self.future_nodes = set()
self.buttons_without_type = []

if "data-mode" in attributes:
    self.modes.add(attributes["data-mode"])
if "data-tour" in attributes:
    self.tours.add(attributes["data-tour"])
if "data-capability" in attributes:
    self.capabilities.add(attributes["data-capability"])
if "future" in attributes.get("class", "").split():
    self.future_sections += 1
if "data-node" in attributes and attributes.get("data-scope") == "c":
    self.future_nodes.add(attributes["data-node"])
if tag == "button" and attributes.get("type") != "button":
    self.buttons_without_type.append(attributes)
```

- [ ] **Step 2: Replace the old filter assertion with the new page contract**

Replace `test_has_filters_and_accessible_detail_region` and add these tests:

```python
def test_has_two_reading_modes_and_four_capabilities(self):
    self.assertEqual({"concept", "ros"}, self.parser.modes)
    self.assertEqual(
        {"perceive", "estimate", "decide", "act"},
        self.parser.capabilities,
    )

def test_has_four_guided_tours(self):
    self.assertEqual(
        {"build-map", "localize", "navigate", "new-obstacle"},
        self.parser.tours,
    )

def test_part_c_is_explicitly_future_and_has_no_invented_nodes(self):
    self.assertGreaterEqual(self.parser.future_sections, 1)
    self.assertEqual(set(), self.parser.future_nodes)
    self.assertIn("NO IMPLEMENTADO", self.html)
    self.assertIn("Parte C", self.html)

def test_accessibility_contract(self):
    self.assertTrue(self.parser.has_detail_region)
    self.assertEqual([], self.parser.buttons_without_type)
    self.assertIn("aria-label=\"Modo de lectura\"", self.html)
    self.assertIn("aria-label=\"Recorridos guiados\"", self.html)
    self.assertIn("prefers-reduced-motion", self.html)
```

Keep `test_contains_all_runtime_nodes`, `test_documents_critical_interfaces`, and `test_is_self_contained` unchanged so the redesign cannot lose technical coverage.

- [ ] **Step 3: Add assertions for the repository invariants**

```python
def test_documents_execution_invariants(self):
    required = (
        "no se ejecutan simultáneamente",
        "productores alternativos",
        "MarkerArray",
        "PoseArray",
        "base_link",
        "base_footprint",
        "odometría densa",
        "único productor de /cmd_vel",
    )
    for text in required:
        with self.subTest(text=text):
            self.assertIn(text, self.html)
```

- [ ] **Step 4: Run the test and confirm the old page fails the new contract**

Run:

```bash
python3 tests/test_architecture_html.py -v
```

Expected: the existing node and self-contained tests pass; the new mode, capability, tour, Part C, and invariant tests fail.

- [ ] **Step 5: Commit the test contract**

```bash
git add tests/test_architecture_html.py
git commit -m "test: define didactic architecture page contract"
```

### Task 2: Build the panoramic semantic map and visual hierarchy

**Files:**
- Modify: `docs/arquitectura_sistema_completo.html`
- Test: `tests/test_architecture_html.py`

- [ ] **Step 1: Replace the stale header and toolbar**

Use this semantic skeleton inside `<body>`:

```html
<div class="page">
  <header class="hero">
    <p class="eyebrow">TP Final · Robótica Autónoma · ROS 2</p>
    <h1>Cómo un robot transforma sensores en movimiento</h1>
    <p class="subtitle">El trabajo construye conocimiento del entorno, lo usa
      para ubicarse y decide cómo llegar a un objetivo.</p>
    <div class="status-strip" aria-label="Estado del trabajo">
      <span class="done">Parte A · implementada</span>
      <span class="done">Parte B · implementada</span>
      <span class="future-status">Parte C · futura, no implementada</span>
    </div>
  </header>

  <section class="mental-model" aria-labelledby="mental-model-title">
    <h2 id="mental-model-title">La idea que une todo</h2>
    <div class="capability-cycle">
      <article data-capability="perceive"><b>Percibir</b><span>¿Qué ve el robot?</span></article>
      <article data-capability="estimate"><b>Estimar</b><span>¿Dónde está y cómo es el entorno?</span></article>
      <article data-capability="decide"><b>Decidir</b><span>¿Por dónde conviene ir?</span></article>
      <article data-capability="act"><b>Actuar</b><span>¿Qué movimiento ejecuta?</span></article>
    </div>
  </section>

  <nav class="controls" aria-label="Controles del mapa">
    <div class="segmented" aria-label="Modo de lectura">
      <button type="button" data-mode="concept" aria-pressed="true">Entender el sistema</button>
      <button type="button" data-mode="ros" aria-pressed="false">Arquitectura ROS</button>
    </div>
    <div class="tours" aria-label="Recorridos guiados">
      <button type="button" data-tour="build-map">Cómo se construye el mapa</button>
      <button type="button" data-tour="localize">Cómo sabe dónde está</button>
      <button type="button" data-tour="navigate">Cómo llega al objetivo</button>
      <button type="button" data-tour="new-obstacle">Qué pasa ante un obstáculo</button>
    </div>
  </nav>

  <main class="workspace">
    <section class="system-map" aria-label="Mapa panorámico del sistema"></section>
    <aside class="details" aria-live="polite" aria-atomic="true"></aside>
  </main>
</div>
```

Do not carry over the branch/commit notice or the old phase filter buttons.

- [ ] **Step 2: Encode the implemented A and B lanes in one panorama**

Inside `.system-map`, create three phase sections. Every implemented node remains a `<button type="button" class="node">` with its exact `data-node` identifier and either `data-scope="a"` or `data-scope="b"`. Group the nodes under conceptual lanes:

```html
<section class="phase phase-a" aria-labelledby="phase-a-title">
  <header><span class="phase-number">Parte A</span><h2 id="phase-a-title">Construir conocimiento</h2></header>
  <div class="pass"><span>Pasada 1</span>
    <button type="button" class="node" data-node="tf_bridge_node" data-scope="a">TF del rosbag</button>
    <button type="button" class="node" data-node="aruco_detector_node" data-scope="a">Detectar ArUco</button>
    <button type="button" class="node" data-node="graph_slam_node" data-scope="a">Optimizar el grafo</button>
    <output class="artifact">trayectoria.json</output>
  </div>
  <div class="pass"><span>Pasada 2</span>
    <span class="source">rosbag · LIDAR + odometría</span>
    <button type="button" class="node" data-node="occupancy_grid_node" data-scope="a">Construir la grilla</button>
    <output class="artifact">map.yaml + map.pgm</output>
  </div>
</section>

<div class="handoff">
  <b>El mapa es el puente</b>
  <code>map.yaml + map.pgm</code>
  <span>Las partes no se ejecutan simultáneamente.</span>
</div>

<section class="phase phase-b" aria-labelledby="phase-b-title">
  <header><span class="phase-number">Parte B</span><h2 id="phase-b-title">Usar el conocimiento</h2></header>
  <div class="lane">
    <button type="button" class="node" data-node="map_loader" data-scope="b">Cargar el mapa</button>
    <button type="button" class="node" data-node="landmark_publisher" data-scope="b">Publicar referencias</button>
    <button type="button" class="node" data-node="landmark_sensor" data-scope="b">Simular observaciones</button>
    <button type="button" class="node" data-node="mcl_localization" data-scope="b">Localizar con MCL</button>
  </div>
  <div class="lane">
    <button type="button" class="node" data-node="global_planner" data-scope="b">Planificar con A*</button>
    <button type="button" class="node" data-node="state_machine" data-scope="b">Coordinar y controlar</button>
  </div>
  <div class="lane">
    <button type="button" class="node" data-node="obstacle_monitor" data-scope="b">Detectar obstáculos nuevos</button>
  </div>
</section>
```

For every arrow, include both label variants:

```html
<div class="connection topic" data-from="map_loader" data-to="global_planner">
  <span data-label="concept">mapa disponible</span>
  <code data-label="ros">/map · OccupancyGrid</code>
</div>
```

Use `.topic`, `.tf`, and `.file` classes to render solid, dashed, and dotted lines respectively.

- [ ] **Step 3: Add Part C as a future-only phase**

```html
<section class="phase phase-c future" aria-labelledby="phase-c-title">
  <header>
    <span class="phase-number">Parte C</span>
    <span class="future-badge">NO IMPLEMENTADO</span>
    <h2 id="phase-c-title">Llevar el sistema al robot real</h2>
  </header>
  <p>La dirección futura es desplegar la navegación sobre el robot real y
    realizar una misión de percepción activa. Todavía no hay nodos, tópicos ni
    contratos implementados para esta etapa.</p>
</section>
```

Do not use `data-node` anywhere in the Part C section.

- [ ] **Step 4: Replace the stylesheet with responsive panoramic styles**

Implement these layout contracts, expanding the existing color variables rather than adding dependencies:

```css
.system-map { display:grid; grid-template-columns:minmax(42rem,1.15fr) 10rem minmax(42rem,1fr) minmax(18rem,.5fr); gap:1rem; }
.phase-a { --phase:#0f6f78; --phase-soft:#dff3f3; }
.phase-b { --phase:#a64b20; --phase-soft:#fbe8dc; }
.phase-c { --phase:#695b83; --phase-soft:#eeeaf5; }
.future { background:repeating-linear-gradient(135deg,var(--phase-soft),var(--phase-soft) 12px,#fff 12px,#fff 24px); }
.connection.topic { border-top-style:solid; }
.connection.tf { border-top-style:dashed; }
.connection.file { border-top-style:dotted; }
[data-label="ros"] { display:none; }
body[data-mode="ros"] [data-label="concept"] { display:none; }
body[data-mode="ros"] [data-label="ros"] { display:inline; }
.is-muted { opacity:.18; filter:grayscale(.75); }
.is-active { outline:3px solid currentColor; outline-offset:3px; }

@media (max-width: 900px) {
  .workspace { grid-template-columns:1fr; }
  .system-map { grid-template-columns:1fr; }
  .details { position:static; }
  .connection { min-height:2.5rem; border-top:0; border-left:2px solid; }
}

@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after { scroll-behavior:auto !important; transition:none !important; }
}
```

Ensure focus styles and active state are distinguishable without relying only on color.

- [ ] **Step 5: Run the static contract tests**

Run:

```bash
python3 tests/test_architecture_html.py -v
```

Expected: node, interface, mode, capability, Part C, accessibility, and self-contained tests pass. Tour behavior may still lack implementation, but all four tour controls are present.

- [ ] **Step 6: Commit the semantic map**

```bash
git add docs/arquitectura_sistema_completo.html
git commit -m "docs: rebuild architecture page as panoramic map"
```

### Task 3: Implement modes, node detail, highlighting, and guided tours

**Files:**
- Modify: `docs/arquitectura_sistema_completo.html`
- Test: `tests/test_architecture_html.py`

- [ ] **Step 1: Define one state object and complete node explanations**

At the start of the embedded script, use a single state model:

```javascript
const state = { mode: "concept", selectedNode: null, activeTour: null, tourStep: 0 };

const nodeDetails = {
  tf_bridge_node: {
    scope: "Parte A · pasada 1", capability: "Percibir",
    concept: "Vuelve utilizables las transformaciones guardadas en el rosbag.",
    problem: "Sin una cadena TF estándar, cámara, robot y mapa no comparten referencias.",
    inputs: ["/tb4_0/tf", "/tb4_0/tf_static"], outputs: ["/tf", "/tf_static"],
    absence: "El detector y RViz no podrían ubicar correctamente las observaciones."
  },
  aruco_detector_node: {
    scope: "Parte A · pasada 1", capability: "Percibir",
    concept: "Convierte imágenes de cámara en observaciones de marcadores ArUco.",
    problem: "El grafo necesita referencias reconocibles para corregir la deriva de odometría.",
    inputs: ["imagen RGB", "CameraInfo o calibración", "TF cámara → base_link"],
    outputs: ["/aruco_detections", "/aruco/debug_image"],
    absence: "Graph SLAM sólo tendría odometría y no podría cerrar el error con landmarks."
  },
  graph_slam_node: {
    scope: "Parte A · pasada 1", capability: "Estimar",
    concept: "Combina movimiento y reobservaciones para corregir poses y landmarks.",
    problem: "La odometría acumula deriva; el grafo busca una trayectoria globalmente consistente.",
    inputs: ["tb4_0/odom", "/aruco_detections", "TF cámara → base_link"],
    outputs: ["TF map → odom", "/trajectory_optimized", "trayectoria.json"],
    absence: "El mapa posterior se proyectaría sobre una trayectoria sin corregir."
  },
  occupancy_grid_node: {
    scope: "Parte A · pasada 2", capability: "Estimar",
    concept: "Proyecta el LIDAR usando la corrección SLAM y la odometría densa.",
    problem: "Hay que transformar mediciones dispersas en una grilla persistente.",
    inputs: ["tb4_0/scan", "tb4_0/odom", "trayectoria.json"],
    outputs: ["/map", "map.pgm", "map.yaml"],
    absence: "Parte B no tendría un mapa estático para localizar ni planificar."
  },
  map_loader: {
    scope: "Parte B", capability: "Estimar",
    concept: "Carga el mapa de Parte A y lo mantiene disponible con QoS latched.",
    problem: "Todos los consumidores necesitan la misma representación estática del entorno.",
    inputs: ["map.yaml", "map.pgm"], outputs: ["/map · OccupancyGrid"],
    absence: "MCL, A* y el monitor no tendrían referencia espacial compartida."
  },
  landmark_publisher: {
    scope: "Parte B", capability: "Percibir",
    concept: "Publica 36 referencias virtuales fijas y conocidas.",
    problem: "MCL necesita saber dónde están los landmarks contra los que compara observaciones.",
    inputs: ["config/landmarks.yaml"], outputs: ["/landmarks · PoseArray", "/landmarks_markers"],
    absence: "Las observaciones no podrían asociarse con posiciones conocidas del mapa."
  },
  landmark_sensor: {
    scope: "Parte B", capability: "Percibir",
    concept: "Emula alcance, bearing, FOV, oclusión y ruido de una cámara de landmarks.",
    problem: "Gazebo no provee directamente el sensor visual requerido por el sistema elegido.",
    inputs: ["/scan", "/landmarks", "TF odom → base_footprint"],
    outputs: ["/observed_landmarks"],
    absence: "MCL avanzaría sólo con odometría y no corregiría su pose global."
  },
  mcl_localization: {
    scope: "Parte B", capability: "Estimar",
    concept: "Mantiene una nube de partículas y estima la pose global del robot.",
    problem: "La navegación necesita reconciliar odometría local con el mapa global.",
    inputs: ["/odom", "/landmarks", "/observed_landmarks", "/initialpose"],
    outputs: ["/mcl_pose", "/particlecloud", "TF map → odom"],
    absence: "Planner, monitor y controlador no sabrían dónde está el robot en el mapa."
  },
  global_planner: {
    scope: "Parte B", capability: "Decidir",
    concept: "Busca una ruta segura con A* 8-conexo sobre la grilla inflada.",
    problem: "Hay que convertir origen y destino en una secuencia transitable.",
    inputs: ["/map", "/plan_request", "TF map → base_footprint"],
    outputs: ["/plan", "/plan_status"],
    absence: "La máquina de estados no tendría un camino global que seguir."
  },
  obstacle_monitor: {
    scope: "Parte B", capability: "Percibir",
    concept: "Compara el scan con el mapa para distinguir obstáculos nuevos de paredes conocidas.",
    problem: "El mundo puede contener objetos que no estaban cuando se construyó el mapa.",
    inputs: ["/scan", "/map", "TF map → base_footprint"],
    outputs: ["/obstacle_detected"],
    absence: "El controlador continuaría una ruta aunque aparezca un objeto no mapeado."
  },
  state_machine: {
    scope: "Parte B", capability: "Decidir y actuar",
    concept: "Coordina planificación, seguimiento, evasión, replanificación y orientación final.",
    problem: "Las conductas deben compartir estado y un único dueño de la velocidad.",
    inputs: ["/goal_pose", "/plan", "/plan_status", "/obstacle_detected", "TF map → base_footprint"],
    outputs: ["/plan_request", "/nav_state", "/cmd_vel"],
    absence: "Nadie convertiría el plan en movimiento ni coordinaría los cambios de conducta."
  }
};
```

- [ ] **Step 2: Implement mode switching without rebuilding the map**

```javascript
function setMode(mode) {
  state.mode = mode;
  document.body.dataset.mode = mode;
  document.querySelectorAll("[data-mode]").forEach(button => {
    button.setAttribute("aria-pressed", String(button.dataset.mode === mode));
  });
  if (state.selectedNode) renderDetails(state.selectedNode);
}

document.querySelectorAll("[data-mode]").forEach(button => {
  button.addEventListener("click", () => setMode(button.dataset.mode));
});
```

- [ ] **Step 3: Implement selection and connection highlighting**

```javascript
function clearHighlight() {
  document.querySelectorAll(".node, .connection").forEach(element => {
    element.classList.remove("is-active", "is-muted");
  });
}

function highlightNodes(nodeIds) {
  const active = new Set(nodeIds);
  document.querySelectorAll(".node").forEach(node => {
    node.classList.toggle("is-active", active.has(node.dataset.node));
    node.classList.toggle("is-muted", !active.has(node.dataset.node));
    node.setAttribute("aria-pressed", String(active.has(node.dataset.node)));
  });
  document.querySelectorAll(".connection").forEach(connection => {
    const connected = active.has(connection.dataset.from) || active.has(connection.dataset.to);
    connection.classList.toggle("is-active", connected);
    connection.classList.toggle("is-muted", !connected);
  });
}

function selectNode(nodeId) {
  state.activeTour = null;
  state.selectedNode = nodeId;
  highlightNodes([nodeId]);
  renderDetails(nodeId);
}
```

Bind each `.node` click to `selectNode(button.dataset.node)`. Clicking the selected node again clears the selection and returns the detail panel to its introductory state.

- [ ] **Step 4: Render the six-question detail panel safely**

Use `textContent` and `replaceChildren`; never interpolate node data with `innerHTML`:

```javascript
function listItems(values) {
  return values.map(value => {
    const item = document.createElement("li");
    item.textContent = value;
    return item;
  });
}

function renderDetails(nodeId) {
  const info = nodeDetails[nodeId];
  detail.title.textContent = nodeId;
  detail.scope.textContent = `${info.scope} · ${info.capability}`;
  detail.concept.textContent = info.concept;
  detail.problem.textContent = info.problem;
  detail.inputs.replaceChildren(...listItems(info.inputs));
  detail.outputs.replaceChildren(...listItems(info.outputs));
  detail.absence.textContent = info.absence;
}
```

- [ ] **Step 5: Define and implement the four guided tours**

```javascript
const tours = {
  "build-map": ["tf_bridge_node", "aruco_detector_node", "graph_slam_node", "occupancy_grid_node"],
  localize: ["map_loader", "landmark_publisher", "landmark_sensor", "mcl_localization"],
  navigate: ["mcl_localization", "global_planner", "state_machine"],
  "new-obstacle": ["obstacle_monitor", "state_machine", "global_planner"]
};

function startTour(tourId) {
  state.activeTour = tourId;
  state.tourStep = 0;
  showTourStep();
}

function showTourStep() {
  const path = tours[state.activeTour];
  const nodeId = path[state.tourStep];
  highlightNodes(path);
  renderDetails(nodeId);
  tourStatus.textContent = `Paso ${state.tourStep + 1} de ${path.length}`;
  tourPrev.disabled = state.tourStep === 0;
  tourNext.textContent = state.tourStep === path.length - 1 ? "Finalizar" : "Siguiente";
}

function advanceTour(delta) {
  const path = tours[state.activeTour];
  const next = state.tourStep + delta;
  if (next >= path.length) return stopTour();
  state.tourStep = Math.max(0, next);
  showTourStep();
}

function stopTour() {
  state.activeTour = null;
  state.tourStep = 0;
  clearHighlight();
  renderIntroduction();
}
```

Add visible `Anterior`, `Siguiente/Finalizar`, and `Salir del recorrido` buttons to the detail panel and wire them to `advanceTour(-1)`, `advanceTour(1)`, and `stopTour()`.

- [ ] **Step 6: Add contextual invariant callouts**

Place short callouts next to their affected phase and include these exact facts:

```html
<p>Graph SLAM y MCL son productores alternativos de <code>map → odom</code>.</p>
<p><code>/landmarks</code> es <code>MarkerArray</code> en A y <code>PoseArray</code> en B.</p>
<p>La segunda pasada usa corrección SLAM y odometría densa; no interpola sólo entre keyframes.</p>
<p><code>state_machine</code> es el único productor de <code>/cmd_vel</code> en Parte B.</p>
```

Also retain the `base_link`/`base_footprint` distinction and the simulated `odom` truth-frame explanation.

- [ ] **Step 7: Run the complete document test**

Run:

```bash
python3 tests/test_architecture_html.py -v
```

Expected: all tests pass.

- [ ] **Step 8: Commit the interactions**

```bash
git add docs/arquitectura_sistema_completo.html
git commit -m "docs: add architecture modes and guided tours"
```

### Task 4: Verify behavior, accessibility, and repository safety

**Files:**
- Modify if a defect is found: `docs/arquitectura_sistema_completo.html`
- Modify if the contract missed a regression: `tests/test_architecture_html.py`

- [ ] **Step 1: Run repository-level static verification**

Run from the repository root:

```bash
python3 tests/test_architecture_html.py -v
python3 tp_final_ws/src/tp_b_navigation/test/test_portable_paths.py -v
python3 -m compileall -q tp_final_ws/src/tp_b_navigation tp_final_ws/src/tp_slam_aruco
git diff --check
```

Expected: both Python test commands pass, `compileall` prints nothing and exits 0, and `git diff --check` prints nothing.

- [ ] **Step 2: Inspect the page at desktop width**

Open `docs/arquitectura_sistema_completo.html` in the in-app browser at approximately 1440×900 and verify:

- The four-capability mental model is visible before the map.
- A, the map handoff, B, and future C read left-to-right.
- Part C is visibly patterned and says `NO IMPLEMENTADO`.
- The detail panel remains readable while navigating the panorama.
- There are no JavaScript console errors.

- [ ] **Step 3: Exercise interactions with authoritative state checks**

In the browser:

1. Select `graph_slam_node`; confirm it and its connections are active and the detail heading changes.
2. Switch to `Arquitectura ROS`; confirm `body[data-mode="ros"]`, the same selected node, and ROS labels are visible.
3. Start each tour and exercise Previous, Next, Finish, and Exit.
4. Confirm every tour returns the page to a neutral state.
5. Tab through all controls; confirm focus is visible and Enter/Space activates buttons.

- [ ] **Step 4: Inspect the responsive layout**

Resize to approximately 390×844 and verify:

- No essential explanation requires horizontal scrolling.
- Phases stack A → handoff → B → C.
- Connections become vertical indicators.
- The detail panel follows the map instead of remaining sticky.
- Mode and tour controls wrap without clipping.

- [ ] **Step 5: Fix any observed defect and rerun verification**

For each defect, first add or strengthen a structural assertion in `tests/test_architecture_html.py` when feasible, confirm it fails, make the minimal HTML/CSS/JS correction, and rerun the four commands from Task 4 Step 1.

- [ ] **Step 6: Commit final polish only if changes were needed**

```bash
git add docs/arquitectura_sistema_completo.html tests/test_architecture_html.py
git commit -m "docs: polish didactic architecture experience"
```

If browser QA required no changes, skip this commit.
