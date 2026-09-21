# Trasformazione della superficie in world

Il nuovo nodo `pear_world_transformer.py` riceve il PointStamped camera-frame,
usa TF2 `world <- table_camera_link_optical` al timestamp originale e pubblica:

- `/orchard/vision/surface_point_world`: PointStamped in metri, frame `world`.
- `/orchard/vision/surface_marker_world`: marker ciano.
- `/orchard/vision/world_status`: validità, diagnostica e contatore trasformazioni.

Non stima il centro, non pubblica `/harvest/pear_pose`, non comanda il robot.
Nessuna coordinata nota della pera entra nel calcolo.
La posa Gazebo è solo un riferimento indipendente di valutazione.

## Aggiornamento e test

Chiudere il vecchio launch. Da PowerShell nella cartella del progetto:

```powershell
powershell -ExecutionPolicy Bypass -File vision\copy_hsv_to_container.ps1 -ContainerId 5f8ebe78a7a6
```

Lo script personale resta ignorato da Git. I collaboratori possono aggiornare
tramite Git o copiare i file, senza usare questo script.
Nel container (su Linux adattare il percorso al proprio workspace):

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbt08pkg --symlink-install
source install/setup.bash
/usr/bin/python3 vision/check_world_runtime.py
ros2 launch rbt08pkg vision_test.launch.py
```

Il preflight usa input/output e topic TF privati, con frame TF univoci:
nessun comando al robot e nessuna modifica al mondo. Verifica il nodo reale con una rotazione
nota di 90 gradi e una traslazione, conservazione timestamp, arrivo ritardato
del TF, reset all'indietro di `/clock`, timeout, recupero e propagazione
dell'invalidazione sorgente. Il reset verifica che la TF statica della camera
non venga persa durante l'avvio di Gazebo.
Controlla inoltre che le subscription TF e clock restino registrate nel nodo
e che il clock ROS non venga sostituito dal clock monotono dei watchdog.
Atteso: `WORLD PREFLIGHT OK`.
Richiede ROS Jazzy, `tf2_ros` e `tf2_geometry_msgs` già dichiarati in package.xml.
Non serve il venv YOLO. Se mancanti, installare i corrispondenti pacchetti
`ros-jazzy-tf2-ros` e `ros-jazzy-tf2-geometry-msgs` (sudo fuori dal container root).

In terminali separati:

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 topic echo /orchard/vision/world_status
```

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 topic echo /orchard/vision/surface_point_world
```

Atteso: `state: VALID`, `valid: true`, contatore crescente, frame `world` e
timestamp uguale al punto camera-frame corrispondente.

## Verifica visiva e riferimento Gazebo

In camera.rviz, Fixed Frame `world`, attivare **Pear surface world**.
La sfera ciano, più piccola, è nello stesso punto della sfera fucsia camera-frame.
Poiché i marker si coprono parzialmente, attivarli alternativamente:
non devono spostarsi passando da uno all'altro, a scena ferma.
Se il display manca: Add -> Marker, topic `/orchard/vision/surface_marker_world`.

Verificare inoltre che il punto coincida con la superficie visibile della pera.
Due marker sovrapposti verificano la trasformazione ma non bastano a dimostrare
una calibrazione world-camera corretta: entrambi usano lo stesso albero TF.
Il modello reale in Gazebo e le sue pose indipendenti servono per questa verifica.
Con pera ancora sul ramo, usare la posa iniziale nel world SDF come riferimento;
se si è mossa, usare la posa corrente di Gazebo, non quella iniziale.
La posizione `<pose>` del modello è l'origine del modello, non necessariamente
il centro della geometria: considerare anche gli offset di link e visual.
Il punto osservato deve stare sul lato rivolto alla camera, a una distanza
compatibile con il raggio locale del frutto dal suo centro, NON coincidere
numericamente con il centro. Non usare questa differenza come errore di misura.

Con la configurazione attuale, il punto camera gia' osservato vicino a
`(-0.002, 0.000, 1.304)` e la TF corrente producono approssimativamente
`(0.809, 0.491, 1.325)` in `world`. Rispetto al centro iniziale Gazebo
`(0.850, 0.550, 1.300)`, la distanza e' circa 0.076 m: coerente con il raggio
0.075 m del corpo della pera. Questi numeri sono solo un controllo per questa
specifica posa; cambiano se si sposta camera o pera.

## Diagnosi e limiti

- `WAITING_POINT`: nessun punto ricevuto; controllare depth_status.
- `WAITING_TF`: TF non disponibile al timestamp della misura; leggere reason.
  Il nodo riprova senza bloccare callbacks, fino al timeout, conservando solo
  l'ultimo punto. Nessun fallback a TF latest.
- `VALID`: punto trasformato; resta necessario verificarne la freschezza.
- `SOURCE_INVALID`: lo stimatore camera ha eliminato il marker sorgente;
  il marker world viene eliminato e il punto in attesa cancellato.
- `INVALID_POINT` / `INVALID_TF`: frame o coordinate non validi.
- `STREAM_TIMEOUT`: nessun nuovo punto da oltre 10 secondi reali.

In caso invalido nessun nuovo punto e marker DELETE. Un vecchio messaggio
PointStamped ricevuto da un consumer non viene cancellato: qualità e sicurezza
per il robot restano una fase successiva. I watchdog camera/world sono separati;
l'invalidazione sorgente è trasmessa anche mediante Marker.DELETE.

Controllo TF indipendente:

```bash
ros2 run tf2_ros tf2_echo world table_camera_link_optical
```

Non avviare un nuovo TF statico per nascondere un errore: verificare quello
già pubblicato dalla camera. Per disattivare solo il nodo world:
`ros2 launch rbt08pkg vision_test.launch.py transform_world:=false`.
