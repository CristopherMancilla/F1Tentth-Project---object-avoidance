# F1Tenth-Repository

**[English](#english) | [Español](#español)**

---

<a name="english"></a>
# English

ROS 2 Humble workspace with the F1TENTH gym simulator, the **Sepang** track and a reactive **Follow the Gap** controller (`gap_node`). Tested on **Ubuntu 22.04**.

- The controller drives the car around Sepang using only the LiDAR. It uses a *disparity extender*: every wall edge is widened by half the car width plus a safety margin, so the car does not cut corners or hit the walls. Steering aims at a point that moves further ahead with speed, and speed changes are gradual (hard braking at high speed makes the car spin).
- Each lap targets **60 s** and must not take more than **75 s**.
- After **10 laps** the car stops and the terminal prints a summary: the data of every lap, the **winning (fastest) lap**, the **slowest lap**, total time, total distance, average and top speed, and total near-collisions.
- RViz shows the **start line in green** and the **car trajectory in red**. The red line is erased and starts again every time the car crosses the start line.

## Repository layout

```
F1Tenth-Repository/                        # this repository = the colcon workspace
└── src/
    ├── controllers/
    │   └── controllers/gap_node.py         # Follow the Gap controller (also pid, ttc, purepursuit, rrt, mpc nodes)
    ├── f1tenth_gym_ros/
    │   ├── config/sim.yaml                 # simulator settings (map, start pose)
    │   ├── f1tenth_gym_ros/lap_visualizer.py  # green start line + red trajectory in RViz
    │   ├── launch/gym_bridge_launch.py     # starts the simulator, RViz and lap_visualizer
    │   └── maps/Sepang_map.png / .yaml     # Sepang map
    ├── path_planning/
    └── waypoints/
```

## 1. Requirements

- Ubuntu 22.04
- ROS 2 Humble (desktop install, includes RViz). Follow the official guide:
  <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

Install the tools used in the next steps:

```bash
sudo apt update
sudo apt install -y git python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
  ros-humble-ackermann-msgs ros-humble-joint-state-publisher ros-humble-xacro
```

Load ROS 2 automatically in every new terminal (once):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Install the F1TENTH gym

The simulator core is a Python package that `f1tenth_gym_ros` uses:

```bash
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .
```

## 3. Download the repository

The repository is already a colcon workspace (it contains `src/`), so clone it directly as the workspace folder:

```bash
cd $HOME
git clone https://github.com/CristopherMancilla/F1Tentth-Project---object-avoidance.git F1Tenth-Repository
cd F1Tenth-Repository
```

The last word of `git clone` is the folder name; with `F1Tenth-Repository` the workspace is `~/F1Tenth-Repository`, the path used in this guide. Use the URL of your own repository or fork if it is different.

Install the ROS dependencies of the packages:

```bash
sudo rosdep init        # only the first time; ignore the error if it was already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## 4. Simulator settings (`sim.yaml`)

File: `src/f1tenth_gym_ros/config/sim.yaml`. Set the **absolute** path of the map (replace `<your_user>`; run `echo $HOME` if you are not sure). The start pose must stay at `(0, 0)` pointing along the main straight, because the start line and the lap counter are placed there:

```yaml
map_path: '/home/<your_user>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_map'   # no extension
map_img_ext: '.png'
sx: 0.0
sy: 0.0
stheta: 0.0
```

The launch file reads the **installed** copy of `sim.yaml`, so build again (step 5) after every change.

## 5. Build (first time and after any change)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To start from scratch: `rm -rf build install log` and build again.

## 6. Launch the simulator (Terminal 1)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Wait until RViz shows the Sepang map, the car and the **green start line**. This also starts the map server, the gym bridge and `lap_visualizer`.

## 7. Run the 10-lap test (Terminal 2)

1. Make sure the car is **on the green start line**. If it was moved or crashed in a previous run, use **2D Pose Estimate** in RViz: click on the green line and drag towards the main straight (+x, to the right). Or restart Terminal 1.
2. In a new terminal:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```

The car drives 10 laps and then stops by itself. A full test takes about 10–12 minutes. Stop everything with `Ctrl+C`.

### Passing criteria

| Criterion | Target |
|---|---|
| Collisions | none (`near-collisions` must be 0 on every lap) |
| Lap time | about 60 s, never more than 75 s |
| Laps | 10, then the car stops |

### Custom parameters

```bash
ros2 run controllers gap_node --ros-args -p target_laps:=3
ros2 run controllers gap_node --ros-args -p target_lap_time:=65.0
ros2 run controllers gap_node --ros-args -p v_max:=10.0 -p safety_margin:=0.30
```

Parameters are read when the node starts; to change one, stop the node (`Ctrl+C`) and run it again.

| Parameter | Default | Meaning |
|---|---|---|
| `target_laps` | 10 | laps before the car stops |
| `target_lap_time` | 60.0 s | desired lap time (`0` disables the regulator) |
| `max_lap_time` | 75.0 s | laps slower than this are marked `TOO SLOW` |
| `v_max` / `v_min` | 14.0 / 2.0 m/s | top speed / lowest speed |
| `max_decel` | 4.0 m/s² | braking used to plan the speed before a wall |
| `brake_margin` | 0.5 m | distance kept before a wall |
| `max_lat_accel` | 12.0 m/s² | lateral acceleration limit in curves |
| `accel_rate` / `decel_rate` | 4.5 / 4.0 m/s² | how fast the speed command may rise / fall |
| `lookahead_min` / `lookahead_gain` / `lookahead_max` | 1.0 m / 0.3 s / 4.0 m | steering aim point: `min + gain × speed`, up to `max` |
| `safety_margin` | 0.25 m | extra clearance added to wall edges |
| `side_clearance` | 0.30 m | the car does not turn towards a side wall closer than this |
| `fov_deg` | 180 | field of view used to choose the direction |
| `start_x`, `start_y`, `start_yaw` | 0.0, 0.0, 0.0 | start line position and driving direction |
| `status_period` | 5.0 s | how often the live status line is printed |

Higher `v_max`, `max_decel`, `max_lat_accel` or `decel_rate` give faster laps but can make the car spin, hit a wall or turn around in the tight hairpins.

## 8. What you will see

During the run, a status line every 5 s and one line per completed lap:

```
[reactive_node]: [Lap 3/10] lap  21.0 s /  124.6 m | total  181.6 s /  1075.0 m | speed  7.1 m/s | cap 14.0 m/s
[reactive_node]: LAP 3/10: 80.19 s | 475.1 m | avg 5.92 m/s | max 8.95 m/s | near-collisions 0
```

After lap 10, the summary. Laps 1–4 are real times measured in the container used for development; the rest of the numbers are an example:

```
==============================================================================
  RACE FINISHED: 10 laps
==============================================================================
  Lap | Time (s) | Dist (m) | Avg (m/s) | Max (m/s) | Near-coll | Status
  --------------------------------------------------------------------------
    1 |    81.23 |    475.2 |      5.85 |      8.90 |         0 | TOO SLOW  <-- SLOWEST
    2 |    80.41 |    475.2 |      5.91 |      8.96 |         0 | TOO SLOW
  ...
    6 |    79.63 |    475.2 |      5.97 |      9.10 |         0 | TOO SLOW  <-- FASTEST
  ...
  --------------------------------------------------------------------------
  Total time:          804.06 s (13.40 min)
  Total distance:      4751.2 m (4.751 km)
  Average speed:       5.91 m/s
  Top speed:           9.10 m/s (lap 6)
  Average lap:         80.41 s (std dev 0.56 s) | average lap distance: 475.1 m
  Laps <= 60 s: 0/10 | laps <= 75 s: 0/10
  Near-collisions:     0
  --------------------------------------------------------------------------
  WINNING LAP (fastest):  lap 6 with 79.63 s (475.2 m, 5.97 m/s average)
  SLOWEST LAP (longest):  lap 1 with 81.23 s (475.2 m, 5.85 m/s average)
  Difference slowest - fastest: 1.60 s
==============================================================================
```

`Status` is `OK (<= target)` for laps up to 60 s, `OK (<= max)` up to 75 s and `TOO SLOW` above 75 s.

In RViz (display `LapMarkers`, topic `/lap_markers`, already in the RViz config):

- **Green line**: start/finish line across the track.
- **Red line**: trajectory of the current lap. It is erased and starts again each time the car crosses the start line. The terminal of Terminal 1 also prints `Lap N completed in X s`.

Lap times are measured with the computer clock. If the PC is slow, the simulation runs slower than real time and the laps take longer. In the development container the simulator runs at about 0.72× real time: laps take about 80 s on the clock (about 58 s of simulated time), so they are marked `TOO SLOW` against the 75 s limit even though the car does not collide. On a faster PC the same laps are shorter.

## Topics

| Topic | Node | Direction | Type |
|---|---|---|---|
| `/scan` | gap_node | subscribes | `sensor_msgs/LaserScan` |
| `/ego_racecar/odom` | gap_node, lap_visualizer | subscribes | `nav_msgs/Odometry` |
| `/drive` | gap_node | publishes | `ackermann_msgs/AckermannDriveStamped` |
| `/lap_markers` | lap_visualizer | publishes | `visualization_msgs/MarkerArray` |

## Troubleshooting

| Problem | Solution |
|---|---|
| `Package '...' not found` | run `source /opt/ros/humble/setup.bash` and `source install/setup.bash` in that terminal |
| `executable 'gap_node' not found` | build and source again (step 5) |
| The old map or old settings are used | you changed `sim.yaml` without building; run `colcon build` again |
| `ModuleNotFoundError: f110_gym` | install the gym again: `cd ~/f1tenth_gym && pip3 install -e .` |
| Missing ROS packages when launching | run the `rosdep install` command of step 3 again |
| RViz closes right after opening | try software rendering: `export LIBGL_ALWAYS_SOFTWARE=1` before launching |
| `libGL error` / `MESA` messages | no GPU access (e.g. inside a container); RViz still works with software rendering |
| The car does not move or stays stuck in a wall | it was not on the start line; reset it with **2D Pose Estimate** and run `gap_node` again |
| Laps are not counted | the car must cross the green line in the driving direction (+x) |
| Laps take more than 75 s | the PC simulates slower than real time; close other programs, or raise `v_max` a little |
| The car touches the walls or spins | lower `v_max`, `max_lat_accel` or `decel_rate`, or raise `safety_margin` |

## Video

Simulation video: <https://www.youtube.com/watch?v=EfybXc3ta54>

---

<a name="español"></a>
# Español

Workspace de ROS 2 Humble con el simulador F1TENTH gym, la pista **Sepang** y un controlador reactivo **Follow the Gap** (`gap_node`). Probado en **Ubuntu 22.04**.

- El controlador conduce el carro por Sepang usando solo el LiDAR. Usa un *disparity extender*: cada borde de pared se ensancha medio ancho del carro más un margen de seguridad, para que el carro no corte las curvas ni choque con las paredes. La dirección apunta a un punto que se aleja con la velocidad, y los cambios de velocidad son graduales (frenar fuerte a alta velocidad hace que el carro trompee).
- Cada vuelta busca durar **60 s** y no debe pasar de **75 s**.
- Al terminar las **10 vueltas** el carro se detiene y la terminal muestra un resumen: los datos de cada vuelta, la **vuelta ganadora (la más rápida)**, la **vuelta más lenta**, tiempo total, distancia total, velocidad promedio y máxima, y total de casi-colisiones.
- RViz muestra la **línea de inicio en verde** y la **trayectoria del carro en rojo**. La línea roja se borra y empieza de nuevo cada vez que el carro cruza la línea de inicio.

## Estructura del repositorio

```
F1Tenth-Repository/                        # este repositorio = el workspace de colcon
└── src/
    ├── controllers/
    │   └── controllers/gap_node.py         # controlador Follow the Gap (también nodos pid, ttc, purepursuit, rrt, mpc)
    ├── f1tenth_gym_ros/
    │   ├── config/sim.yaml                 # configuración del simulador (mapa, pose inicial)
    │   ├── f1tenth_gym_ros/lap_visualizer.py  # línea de inicio verde + trayectoria roja en RViz
    │   ├── launch/gym_bridge_launch.py     # inicia el simulador, RViz y lap_visualizer
    │   └── maps/Sepang_map.png / .yaml     # mapa de Sepang
    ├── path_planning/
    └── waypoints/
```

## 1. Requisitos

- Ubuntu 22.04
- ROS 2 Humble (instalación de escritorio, incluye RViz). Sigue la guía oficial:
  <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

Instala las herramientas que se usan en los siguientes pasos:

```bash
sudo apt update
sudo apt install -y git python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
  ros-humble-ackermann-msgs ros-humble-joint-state-publisher ros-humble-xacro
```

Carga ROS 2 automáticamente en cada terminal nueva (una sola vez):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Instalar el F1TENTH gym

El núcleo del simulador es un paquete de Python que usa `f1tenth_gym_ros`:

```bash
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .
```

## 3. Descargar el repositorio

El repositorio ya es un workspace de colcon (contiene `src/`), así que se clona directamente como la carpeta del workspace:

```bash
cd $HOME
git clone https://github.com/CristopherMancilla/F1Tentth-Project---object-avoidance.git F1Tenth-Repository
cd F1Tenth-Repository
```

La última palabra de `git clone` es el nombre de la carpeta; con `F1Tenth-Repository` el workspace queda en `~/F1Tenth-Repository`, la ruta que usa esta guía. Si tu repositorio o fork tiene otra URL, usa esa.

Instala las dependencias de ROS de los paquetes:

```bash
sudo rosdep init        # solo la primera vez; ignora el error si ya estaba inicializado
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## 4. Configuración del simulador (`sim.yaml`)

Archivo: `src/f1tenth_gym_ros/config/sim.yaml`. Escribe la ruta **absoluta** del mapa (reemplaza `<tu_usuario>`; ejecuta `echo $HOME` si no estás seguro). La pose inicial debe quedar en `(0, 0)` apuntando por la recta principal, porque ahí están la línea de inicio y el contador de vueltas:

```yaml
map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_map'   # sin extensión
map_img_ext: '.png'
sx: 0.0
sy: 0.0
stheta: 0.0
```

El launch lee la copia **instalada** de `sim.yaml`, así que vuelve a compilar (paso 5) después de cada cambio.

## 5. Compilar (la primera vez y después de cualquier cambio)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Para empezar de cero: `rm -rf build install log` y compila de nuevo.

## 6. Lanzar el simulador (Terminal 1)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Espera a que RViz muestre el mapa de Sepang, el carro y la **línea de inicio verde**. Esto también inicia el servidor de mapas, el puente del gym y `lap_visualizer`.

## 7. Ejecutar la prueba de 10 vueltas (Terminal 2)

1. Verifica que el carro esté **sobre la línea de inicio verde**. Si se movió o chocó en una prueba anterior, usa **2D Pose Estimate** en RViz: haz clic sobre la línea verde y arrastra hacia la recta principal (+x, a la derecha). O reinicia la Terminal 1.
2. En una terminal nueva:

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controllers gap_node
```

El carro da 10 vueltas y luego se detiene solo. La prueba completa tarda unos 10–12 minutos. Detén todo con `Ctrl+C`.

### Criterios de aprobación

| Criterio | Objetivo |
|---|---|
| Colisiones | ninguna (`near-collisions` debe ser 0 en cada vuelta) |
| Tiempo por vuelta | unos 60 s, nunca más de 75 s |
| Vueltas | 10, y luego el carro se detiene |

### Parámetros personalizados

```bash
ros2 run controllers gap_node --ros-args -p target_laps:=3
ros2 run controllers gap_node --ros-args -p target_lap_time:=65.0
ros2 run controllers gap_node --ros-args -p v_max:=10.0 -p safety_margin:=0.30
```

Los parámetros se leen al iniciar el nodo; para cambiar uno, detén el nodo (`Ctrl+C`) y vuelve a ejecutarlo.

| Parámetro | Por defecto | Significado |
|---|---|---|
| `target_laps` | 10 | vueltas antes de que el carro se detenga |
| `target_lap_time` | 60.0 s | tiempo deseado por vuelta (`0` desactiva el regulador) |
| `max_lap_time` | 75.0 s | las vueltas más lentas se marcan `TOO SLOW` |
| `v_max` / `v_min` | 14.0 / 2.0 m/s | velocidad máxima / mínima |
| `max_decel` | 4.0 m/s² | frenado usado para planear la velocidad antes de una pared |
| `brake_margin` | 0.5 m | distancia que se deja antes de una pared |
| `max_lat_accel` | 12.0 m/s² | límite de aceleración lateral en curvas |
| `accel_rate` / `decel_rate` | 4.5 / 4.0 m/s² | qué tan rápido puede subir / bajar la velocidad pedida |
| `lookahead_min` / `lookahead_gain` / `lookahead_max` | 1.0 m / 0.3 s / 4.0 m | punto al que apunta la dirección: `min + gain × velocidad`, hasta `max` |
| `safety_margin` | 0.25 m | holgura extra que se agrega a los bordes de pared |
| `side_clearance` | 0.30 m | el carro no gira hacia una pared lateral más cercana que esto |
| `fov_deg` | 180 | campo de visión usado para elegir la dirección |
| `start_x`, `start_y`, `start_yaw` | 0.0, 0.0, 0.0 | posición de la línea de inicio y sentido de la marcha |
| `status_period` | 5.0 s | cada cuánto se imprime la línea de estado |

Subir `v_max`, `max_decel`, `max_lat_accel` o `decel_rate` da vueltas más rápidas, pero el carro puede trompear, chocar o dar la vuelta en las horquillas cerradas.

## 8. Qué verás

Durante la prueba, una línea de estado cada 5 s y una línea por cada vuelta completada:

```
[reactive_node]: [Lap 3/10] lap  21.0 s /  124.6 m | total  181.6 s /  1075.0 m | speed  7.1 m/s | cap 14.0 m/s
[reactive_node]: LAP 3/10: 80.19 s | 475.1 m | avg 5.92 m/s | max 8.95 m/s | near-collisions 0
```

Al terminar la vuelta 10, el resumen. Las vueltas 1–4 son tiempos reales medidos en el contenedor de desarrollo; el resto de los números es un ejemplo:

```
==============================================================================
  RACE FINISHED: 10 laps
==============================================================================
  Lap | Time (s) | Dist (m) | Avg (m/s) | Max (m/s) | Near-coll | Status
  --------------------------------------------------------------------------
    1 |    81.23 |    475.2 |      5.85 |      8.90 |         0 | TOO SLOW  <-- SLOWEST
    2 |    80.41 |    475.2 |      5.91 |      8.96 |         0 | TOO SLOW
  ...
    6 |    79.63 |    475.2 |      5.97 |      9.10 |         0 | TOO SLOW  <-- FASTEST
  ...
  --------------------------------------------------------------------------
  Total time:          804.06 s (13.40 min)
  Total distance:      4751.2 m (4.751 km)
  Average speed:       5.91 m/s
  Top speed:           9.10 m/s (lap 6)
  Average lap:         80.41 s (std dev 0.56 s) | average lap distance: 475.1 m
  Laps <= 60 s: 0/10 | laps <= 75 s: 0/10
  Near-collisions:     0
  --------------------------------------------------------------------------
  WINNING LAP (fastest):  lap 6 with 79.63 s (475.2 m, 5.97 m/s average)
  SLOWEST LAP (longest):  lap 1 with 81.23 s (475.2 m, 5.85 m/s average)
  Difference slowest - fastest: 1.60 s
==============================================================================
```

`Status` es `OK (<= target)` para vueltas de hasta 60 s, `OK (<= max)` hasta 75 s y `TOO SLOW` por encima de 75 s.

En RViz (display `LapMarkers`, tópico `/lap_markers`, ya incluido en la configuración de RViz):

- **Línea verde**: línea de inicio/meta atravesando la pista.
- **Línea roja**: trayectoria de la vuelta actual. Se borra y empieza de nuevo cada vez que el carro cruza la línea de inicio. La Terminal 1 también imprime `Lap N completed in X s`.

Los tiempos de vuelta se miden con el reloj de la computadora. Si la PC es lenta, la simulación corre más lento que el tiempo real y las vueltas tardan más. En el contenedor de desarrollo el simulador corre a unas 0.72× del tiempo real: las vueltas tardan unos 80 s de reloj (unos 58 s de tiempo simulado), así que se marcan `TOO SLOW` frente al límite de 75 s aunque el carro no choque. En una PC más rápida las mismas vueltas duran menos.

## Tópicos

| Tópico | Nodo | Dirección | Tipo |
|---|---|---|---|
| `/scan` | gap_node | se suscribe | `sensor_msgs/LaserScan` |
| `/ego_racecar/odom` | gap_node, lap_visualizer | se suscribe | `nav_msgs/Odometry` |
| `/drive` | gap_node | publica | `ackermann_msgs/AckermannDriveStamped` |
| `/lap_markers` | lap_visualizer | publica | `visualization_msgs/MarkerArray` |

## Solución de problemas

| Problema | Solución |
|---|---|
| `Package '...' not found` | ejecuta `source /opt/ros/humble/setup.bash` y `source install/setup.bash` en esa terminal |
| `executable 'gap_node' not found` | compila y haz `source` de nuevo (paso 5) |
| Se usa el mapa o la configuración anterior | cambiaste `sim.yaml` sin compilar; ejecuta `colcon build` otra vez |
| `ModuleNotFoundError: f110_gym` | instala de nuevo el gym: `cd ~/f1tenth_gym && pip3 install -e .` |
| Faltan paquetes de ROS al lanzar | ejecuta otra vez el `rosdep install` del paso 3 |
| RViz se cierra al abrir | prueba renderizado por software: `export LIBGL_ALWAYS_SOFTWARE=1` antes de lanzar |
| Mensajes `libGL error` / `MESA` | no hay acceso a la GPU (por ejemplo, dentro de un contenedor); RViz funciona igual con renderizado por software |
| El carro no se mueve o queda atascado en una pared | no estaba en la línea de inicio; reinícialo con **2D Pose Estimate** y vuelve a ejecutar `gap_node` |
| No se cuentan las vueltas | el carro debe cruzar la línea verde en el sentido de la marcha (+x) |
| Las vueltas pasan de 75 s | la PC simula más lento que el tiempo real; cierra otros programas o sube un poco `v_max` |
| El carro toca las paredes o trompea | baja `v_max`, `max_lat_accel` o `decel_rate`, o sube `safety_margin` |

## Video

Video de la simulación: <https://www.youtube.com/watch?v=EfybXc3ta54>
