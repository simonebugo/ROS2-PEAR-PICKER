# Fase: superficie 3D nel frame camera

Questa fase non muove il robot e non pubblica `/harvest/pear_pose`.
Il risultato è un punto sulla superficie visibile della pera, non il suo centro.
La trasformazione nel world e la correzione per il grasp restano fasi successive.

## Preparazione nel container

Da PowerShell nella cartella locale del progetto, se usi il tuo script personale:

```powershell
powershell -ExecutionPolicy Bypass -File vision\copy_hsv_to_container.ps1 -ContainerId 5f8ebe78a7a6
```

Lo script copia solo i file elencati: non modifica venv, dataset, pesi o installazioni.
Lo script è ignorato da Git: gli altri collaboratori devono aggiornare i sorgenti
tramite Git o copiarli e ricompilare. Non servono Ultralytics o il venv YOLO.

Nel container:

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbt08pkg --symlink-install
source install/setup.bash
/usr/bin/python3 vision/test_depth.py -v
/usr/bin/python3 vision/check_depth_runtime.py
```

Su Linux fuori dal container sostituire `/root/sr_ws10` con il workspace effettivo,
ad esempio `~/sr_ws10`. Dipendenze già previste dal progetto:
`python3-opencv`, `python3-numpy`, `ros-jazzy-cv-bridge` e messaggi ROS.
I test ROS usano topic privati e dati sintetici; non comandano il robot.
Risultato atteso: test tutti OK e `DEPTH PREFLIGHT OK`.

## Test con camera Gazebo

Chiudere eventuali vecchi launch della camera/visione prima di riavviare,
per evitare publisher e nodi duplicati. Terminale 1:

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 launch rbt08pkg vision_test.launch.py
```

Lo stesso launch ora avvia anche `pear_surface_estimator.py`.
Se la camera è già avviata separatamente:

```bash
ros2 launch rbt08pkg vision_test.launch.py start_camera:=false
```

Terminale 2:

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 topic echo /orchard/vision/depth_status
```

Terminale 3:

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 topic echo /orchard/vision/surface_point
```

Quando la pera è segmentata correttamente, lo stato deve essere `VALID`,
`valid: true`, `paired_frames` crescente, `estimate` con punto, pixel,
profondità mediana, spread e numero di campioni validi.
Il messaggio PointStamped conserva timestamp RGB/depth e frame
`table_camera_link_optical`. Coordinate in metri: X destra, Y basso, Z avanti.
Con scena ferma verificare variazioni contenute, indicativamente millimetri
o pochi centimetri: questa è una verifica empirica, non una precisione garantita.

RViz mostra una piccola sfera magenta tramite **Pear observed surface**.
Deve essere sulla superficie della pera rivolta alla camera, non al centro.
Con Fixed Frame `world` RViz usa i TF già esistenti per visualizzarla:
il nodo continua a pubblicare esclusivamente nel frame camera.
Se non compare, verificare display attivo, stato del display, TF e `depth_status`.

## Come funziona

1. Target mask e depth sono abbinate solo con timestamp esattamente uguali.
   Nessuna profondità viene associata a una maschera di un altro frame.
2. CameraInfo fornisce K e dimensioni; la calibrazione è riutilizzata perché la
   camera attuale è fissa. Sono richiesti RGB/depth registrati, stessa risoluzione
   e frame, immagini senza distorsione (`plumb_bob` con D nullo).
3. Erosione della maschera, selezione del pixel più lontano dal contorno,
   campionamento di un piccolo disco interno.
4. Scarto NaN, infiniti, zero e valori fuori dal range. Verifica quantità,
   frazione di pixel validi e spread robusto P90-P10.
5. Calcolo della mediana, scelta di un campione reale valido più vicino alla
   mediana, e deproiezione usando il pixel di quel campione e gli intrinseci K:
   `X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`.

Non c'è smoothing temporale, per non nascondere movimenti o introdurre ritardo.
La robustezza deriva dai campioni interni e dalla mediana; ulteriori filtri
e controlli di stabilità saranno nella fase qualità/sicurezza.

Configurazione: `rbt08pkg/config/pear_depth.yaml`.
Non alzare le tolleranze alla cieca: prima controllare maschera, registrazione e depth.
Per disattivare solo questa nuova fase: `estimate_3d:=false` nel launch.

## Stati e diagnosi

- `WAITING_PAIR`: attesa calibrazione o prima coppia esatta. Controllare
  `camera_info_received`, `pending_masks`, `pending_depth` e topic camera.
- `VALID`: punto appena stimato e utilizzabile solo con controllo di freschezza.
- `NO_TARGET`: maschera abbinata vuota; nessun nuovo punto.
- `INVALID_DEPTH`: encoding, calibrazione, dimensioni o campioni non validi;
  leggere `reason`.
- `INVALID_FRAME`: frame del messaggio diverso da quello configurato.
- `STALE_PAIR`: coppia rimasta in attesa oltre il timeout.
- `TIME_RESET`: timestamp tornato indietro, cache cancellata per reset simulazione.
- `STREAM_TIMEOUT`: nessuna coppia recente, dopo 10 secondi reali di default.

In caso non valido il nodo elimina il marker e smette di pubblicare punti;
un vecchio PointStamped già ricevuto non viene cancellato. I futuri consumer
devono controllare stato e freschezza prima di avviare il robot.
Se i topic arrivano ma non si abbinano, confrontare gli header di target_mask
e depth, verificare l'assenza di publisher duplicati e il carico del simulatore.
