# Fase 1: collaudo camera RGB-D

Camera fissa in world a `(0.55, 1.35, 1.55)` m, rivolta verso la posa
iniziale della pera `(0.85, 0.55, 1.30)` m. RGB e depth condividono
risoluzione 640x480, FOV orizzontale 80 gradi, 10 Hz e frame ottico
`table_camera_link_optical`. Depth: `32FC1`, metri; fuori portata i valori
possono essere non finiti. CameraInfo vale per entrambi gli stream.

## Avvio nel container

Chiudere i launch Gazebo precedenti per evitare mondi o controller duplicati.

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbt08pkg --symlink-install
source install/setup.bash
ros2 launch rbt08pkg camera_test.launch.py
```

Il launch avvia Gazebo, robot e controller per tenerlo stabile, bridge camera
e RViz. La pera resta sul ramo. Non avvia MoveIt o il pick-and-place.

## Controlli in un secondo terminale

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 topic list | grep /orchard/camera
ros2 topic echo /orchard/camera/camera_info --once
ros2 topic echo /orchard/camera/rgb/image_raw --field encoding --once
ros2 topic echo /orchard/camera/depth/image_raw --field encoding --once
ros2 topic hz /orchard/camera/rgb/image_raw
```

Interrompere `topic hz` con Ctrl+C, quindi:

```bash
ros2 run tf2_ros tf2_echo world table_camera_link_optical
```

Atteso: tre topic, immagini 640x480, RGB `rgb8`, depth `32FC1`, matrice K
non nulla, TF disponibile. Il frame_id dei tre messaggi deve essere
`table_camera_link_optical`. La frequenza e' circa 10 Hz di simulazione:
se il real-time factor e' inferiore a 1, topic hz misurera' meno di 10 Hz.

In RViz i pannelli RGB e Depth devono aggiornarsi; la pera deve essere
visibile vicino al centro. Il depth mostra la superficie visibile della
pera, non il centro geometrico: la correzione per il punto di grasp sara'
validata nella futura fase di stima 3D.

## Se non arrivano immagini

```bash
gz topic -l | grep table_camera
```

Se mancano anche i topic Gazebo, controllare errori del plugin Sensors e di
Ogre2/OpenGL nel terminale di avvio. Ogre2 sul server serve alla depth:
cambiare soltanto il renderer GUI non elimina questo requisito.

Se i topic Gazebo esistono, controllare `ros2 node info /camera_bridge`
e gli errori di conversione del bridge. La configurazione e' installata
in `share/rbt08pkg/config/gz_bridge_camera.yaml`.

Per una GUI piu' compatibile con la VM:

```bash
ros2 launch rbt08pkg camera_test.launch.py render_engine:=ogre
```

Per eseguire senza RViz: aggiungere `rviz:=false`.

Approvare la fase dopo aver verificato RGB, depth, intrinseci, TF e
inquadratura. La sorgente statica della pera continua a essere il riferimento
del pick-and-place; in questa fase non si pubblica una posa da visione.

Riferimenti: [bridge RGB-D ufficiale Gazebo](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/launch/rgbd_camera.launch.py)
e [implementazione RGB-D di Gazebo Sensors 8](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/RgbdCameraSensor.cc).
