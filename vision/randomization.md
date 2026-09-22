# Randomizzazione iniziale di ramo e pera

Il generatore viene eseguito prima di Gazebo. Estrae valori continui uniformi
con un generatore locale legato al seed; non sceglie da una lista di pose.
I limiti sono in `rbt08pkg/config/orchard_randomization.json`:

| Parametro | Intervallo iniziale | Significato |
| --- | --- | --- |
| `branch_height_m` | 1.44–1.52 m | Altezza del punto di attacco al tronco, in world |
| `branch_yaw_deg` | −12°–12° | Deviazione dal ramo rivolto verso −X; positivo verso −Y |
| `branch_elevation_deg` | −5°–5° | Inclinazione; positivo alza l'estremità del ramo |
| `pear_distance_m` | 0.44–0.55 m | Distanza dal tronco lungo il ramo di 0.60 m |

La pera resta verticale; la punta del gambo si trova sul punto scelto del ramo.
Si modificano solo il ramo inferiore e la pera. Camera, tronco, ramo superiore,
cestino e robot mantengono la configurazione precedente.

## Selezione e limiti della verifica

Ogni candidato deve stare davanti al tronco e rientrare nell'inquadratura
della camera attuale con un margine ai bordi (20 pixel). Il filtro considera
corpo, collo e gambo e richiede un diametro del corpo di almeno 20 pixel.
Questo verifica il campo visivo geometrico, non l'assenza di occlusioni.

La catena cinematica viene letta dai giunti numerici degli URDF del robot e del
gripper. Un filtro IK numerico verifica limiti articolari, errore di posizione
inferiore a 0.5 mm ed errore angolare inferiore a 0.003 rad per pre-grasp,
metà avvicinamento, grasp, metà sollevamento e lift. Usa gli offset attuali
del manager: 25 cm di arretramento e 20 cm di sollevamento.
Queste soluzioni sono salvate per diagnosi; non vengono eseguite sul robot.

Un candidato respinto viene sostituito dalla successiva estrazione dello stesso
generatore. Dopo 40 tentativi l'avvio fallisce con il motivo, senza ripiegare
silenziosamente sulla vecchia posa. I limiti iniziali sono prudenti e passano
il filtro cinematico locale; non sono una certificazione di assenza di collisioni
dell'intero braccio. Le mesh e le traiettorie si verificano con MoveIt e Gazebo.
Anche il preflight ROS non sostituisce il test fisico di presa e deposito.

## Un solo scenario per Gazebo e MoveIt

Il launch stampa:

```text
ORCHARD seed=42, attempt=1, pear=[...]
ORCHARD scenario=/tmp/pear_orchard_42_.../scenario.json
```

In quella cartella vengono creati un SDF e un manifest JSON; il world originale
nel progetto non viene sovrascritto. Il SDF contiene le nuove pose del ramo
(visuale e collisione) e della pera, prima dell'inizializzazione dei detachable
joint. `spawn_planning_scene.py` legge lo stesso manifest per allineare le
collisioni MoveIt del ramo, corpo e collo della pera.

Il manifest contiene seed, limiti, valori estratti, pose, soluzioni IK e hash
dei file di partenza. Per ripetere un caso usare lo stesso seed con gli stessi
file, limiti e versione del generatore. `seed:=auto` sceglie un nuovo seed e lo
stampa. Il seed riproduce lo scenario iniziale, non rumore della camera,
pianificazione OMPL o tempistiche della simulazione.

Nella scena MoveIt il ramo che sostiene la pera ha un raggio maggiorato di
15 mm rispetto alla collisione fisica di Gazebo. Il planner deve quindi
lasciare un piccolo spazio al gripper durante arretramento e lift. Il preflight
controlla esplicitamente questa differenza intenzionale.
I file temporanei vengono lasciati per consultazione e non sono versionati.

`pick_place_manager.py` non riceve il manifest né le coordinate note.
`/harvest/pear_pose` proviene sempre dalla stima RGB-D. La posa nota è usata
per collisioni e valutazione; la randomizzazione rifiuta `pose_source:=static`.

## Copia, build e test

Chiudere i launch precedenti. In PowerShell sul PC:

```powershell
cd D:\Matteo\Desktop\sr_ws10
powershell -ExecutionPolicy Bypass -File vision\copy_hsv_to_container.ps1 -ContainerId 5f8ebe78a7a6
```

Nel container, con Python di sistema e senza venv:

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbt08pkg rbt10pkg_mit --symlink-install
source install/setup.bash
/usr/bin/python3 vision/test_randomization.py -v
/usr/bin/python3 vision/test_motion_checks.py -v
```

Il filtro richiede NumPy, già dipendenza del pacchetto e della pipeline depth.
Non richiede SciPy o pacchetti di training.

## Avvio e preflight

Terminale 1:

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=vision randomize:=true seed:=42
```

Il comando precedente, senza `randomize:=true`, conserva lo scenario fisso.
Per un nuovo scenario ripetere un avvio pulito con `seed:=43` oppure
`seed:=auto`. Non fare reset o teletrasporti a simulazione già avviata.

Terminale 2: attendere scena e controllori pronti, poi sostituire il percorso
qui sotto con **quello stampato da questo avvio**, prima di inviare il trigger:

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 control list_controllers
/usr/bin/python3 vision/check_randomized_runtime.py /tmp/pear_orchard_42_XXXX/scenario.json
```

Il test non muove il robot. Verifica che:

- collisioni di ramo e pera in MoveIt corrispondano allo scenario generato;
- MoveIt trovi IK con collisioni per pre-grasp/lift e IK cinematica per
  avvicinamento/grasp, dove il contatto con il frutto è intenzionale;
- una nuova misura del centro e `/harvest/pear_pose` coincidano e siano a meno
  di 2.5 cm dalla posa nota (soglia del test, non precisione garantita).

Se fallisce, conservare seed, manifest e messaggio di errore e non avviare il
ciclo di quel test. Un fallimento della singola richiesta IK non dimostra
l'impossibilità matematica di ogni configurazione: indica che la verifica
MoveIt con il seed articolare fornito non è passata.

## Visualizzazione e ciclo completo

Per vedere le collisioni MoveIt in RViz, da un altro terminale configurato:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix rbt10pkg_mit)/share/rbt10pkg_mit/config/moveit.rviz" --ros-args -p use_sim_time:=true
```

Controllare la planning scene sul topic `/monitored_planning_scene`. Aggiungere
eventualmente il display Marker `/orchard/vision/center_marker_world` per
confrontare il centro arancione con il frutto della scena.
Per le immagini usare invece `camera.rviz`, come descritto in `pick_place_vision.md`.

Dopo preflight e controllo visivo, inviare una sola volta:

```bash
ros2 topic pub --once /harvest/trigger std_msgs/msg/Empty '{}'
```

Attendere il rilascio nel cestino e `SEQUENZA PICK AND PLACE COMPLETATA CON SUCCESSO`.
La raccolta dei risultati dovrebbe riportare seed, errore del centro, esito e
fase di eventuale arresto. Prima di allargare i bounds, validare diversi seed
con questa procedura; una maggiore varietà può aumentare occlusioni e collisioni.

Per usare un file di limiti alternativo:

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=vision randomize:=true seed:=42 randomization_config:=/root/sr_ws10/install/rbt08pkg/share/rbt08pkg/config/orchard_randomization_wide.json
```

Il profilo `orchard_randomization_wide.json` amplia altezza del ramo a
1.40–1.56 m, angolo orizzontale a ±20° e distanza della pera dal tronco a
0.40–0.56 m; mantiene l'inclinazione verticale a ±5°. Per tornare al profilo
precedente, omettere `randomization_config`: il default resta
`orchard_randomization.json`. Ogni profilo richiede un nuovo avvio del launch;
eseguire il preflight con il manifest stampato da quell'avvio prima del trigger.

### Caso limite ripetibile

Per testare un angolo del profilo largo, usare
`orchard_randomization_edge.json`: altezza 1.56 m, yaw +20°, elevazione +5°
e pera a 0.56 m dal tronco. Ogni intervallo ha estremi uguali e
`max_attempts` è 1: il generatore produce esattamente questa geometria oppure
fallisce esplicitamente. Il seed rimane nel manifest, ma non cambia la posa
con questo profilo. Per altri casi limite occorre modificare una copia del file
o selezionare un altro profilo, senza cambiare quello di default.

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=vision randomize:=true seed:=42 randomization_config:=/root/sr_ws10/install/rbt08pkg/share/rbt08pkg/config/orchard_randomization_edge.json
```

Prima del trigger eseguire `check_randomized_runtime.py` con il manifest
stampato dal nuovo launch. Il filtro locale verifica vista e IK, mentre il
preflight verifica allineamento scena, IK MoveIt e stima visiva. La traiettoria
intera e il grasp fisico restano da validare con il ciclo Gazebo.
