# Fase 2: verifica bridge e TF

## Avvio

Aggiornare i sorgenti nel container e chiudere i precedenti launch Gazebo.

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbt08pkg --symlink-install
source install/setup.bash
ros2 launch rbt08pkg camera_test.launch.py render_engine:=ogre
```

Il launch avvia automaticamente `camera_pipeline_check.py`. Attendere:

```text
CAMERA READY: RGB, depth, CameraInfo, timestamp e TF coerenti.
```

In un secondo terminale con l'overlay ROS caricato:

```bash
ros2 topic echo /orchard/camera/status --once
ros2 run tf2_ros tf2_echo world table_camera_link_optical
```

Atteso: `ready: true`, `errors: []`, `paired_frames` maggiore di zero.
Gli avvisi iniziali possono comparire mentre Gazebo e il sensore si avviano.

## Cosa verifica il nodo

- Arrivo di RGB, depth e CameraInfo con `table_camera_link_optical`.
- Encoding RGB `rgb8`, depth `32FC1`, buffer e dimensioni coerenti.
- Matrice K finita, focali positive, dimensioni CameraInfo uguali alle immagini.
- Esistenza di coppie RGB/depth con timestamp identici, prima di introdurre YOLO.
- Assenza di publisher ROS multipli sui tre topic camera.
- Disponibilita' della trasformazione statica `world <- table_camera_link_optical`.
- Nessun flusso fermo per oltre 10 secondi reali: il controllo funziona anche
  con simulazione lenta o in pausa. Una pausa prolungata segnala flusso fermo.

In RViz compare una freccia ciano lunga un metro dall'origine camera nella
direzione ottica +Z, trasformata in world. E' un punto/raggio di prova TF:
non rappresenta una pera rilevata e non misura la sua posizione.

Depth usa gia' la scala validata: Normalize Range disattivato, Min 0.1, Max 5.
Per disabilitare il controllo diagnostico aggiungere `check_camera:=false`.

Il controllo non sincronizza ancora i messaggi per l'inferenza e non verifica
il contenuto metrico di ogni pixel. La precisione della posa e i dati depth
non finiti verranno gestiti nella fase di stima 3D. Per la camera fissa si usa
la TF piu' recente; una futura camera mobile dovra' usare il timestamp immagine.

## Audit dataset Pear640

Percorso verificato: `datasets/pear640/raw/images` e `labels`.
712 immagini, 698 file label, 14 immagini senza label, nessuna label senza
immagine. Le immagini senza label potrebbero essere esempi negativi: occorre
verificare che non contengano pere prima di usarle nel training.
Le label hanno una classe (0) e 8340 righe oggetto; 52 righe hanno
valori geometrici da verificare prima del training. Queste osservazioni non
modificano gli originali e non attestano la qualita' visiva delle annotazioni.
`datasets/` e' escluso da Git e resta fuori dall'installazione colcon.

Validare la fase con CAMERA READY, freccia visibile in world e pannelli RGB/depth
aggiornati. La pipeline non pubblica ancora `/harvest/pear_pose` dalla visione.
