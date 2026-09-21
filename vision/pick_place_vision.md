# Collegamento visione → pick-and-place

Il centro stimato viene convertito in `PoseStamped` dal nodo
`pear_world_transformer`, abilitando `publish_harvest_pose`. Non serve un nuovo
nodo. `/harvest/pear_pose` conserva frame `world` e timestamp della misura;
l'orientamento è identità perché il manager calcola già l'orientamento di presa.
`center_status` contiene lo stesso timestamp e viene pubblicato per ogni nuova
stima, oltre al riepilogo periodico.

Al trigger il manager verifica: posa disponibile, coordinate finite, stato
valido per il medesimo timestamp, età entro `max_pose_age_s` (default 5 secondi).
L'età considera sia il timestamp ROS sia il tempo reale trascorso dalla ricezione,
inclusa l'età riportata dalla sorgente: fermare Gazebo o ripubblicare lo stato
non rende fresca una vecchia misura. Un timestamp futuro, per esempio dopo un
reset del clock, viene rifiutato fino all'arrivo di una nuova posa.
TF deve essere disponibile per esprimere la posa in `base_link`.

Non si aspetta una finestra di stabilità e non si applicano medie temporali.
Il thread della sequenza usa le pose ricevute al trigger; nuove rilevazioni
aggiornano solo il target per un eventuale ciclo successivo.

## Copia e build

Chiudere i launch precedenti prima del test integrato. Da PowerShell sul PC:

```powershell
cd D:\Matteo\Desktop\sr_ws10
powershell -ExecutionPolicy Bypass -File vision\copy_hsv_to_container.ps1 -ContainerId 5f8ebe78a7a6
```

Lo script copia i sorgenti della visione, il manager, i due launch MoveIt e i
test/documenti; non copia né modifica ambienti virtuali. Rimane ignorato da Git.
Nel container, senza venv:

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbt08pkg rbt10pkg_mit --symlink-install
source install/setup.bash
/usr/bin/python3 vision/test_pick_target.py -v
/usr/bin/python3 vision/check_world_runtime.py
/usr/bin/python3 vision/check_pick_target_runtime.py
```

Il nuovo preflight usa topic privati e sostituisce l'esecuzione con una
registrazione delle pose: non comanda il robot. Risultato atteso:
`PICK TARGET PREFLIGHT OK ... Nessun movimento.`

## Avvio completo (terminale 1)

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=vision render_engine:=ogre max_pose_age_s:=5.0
```

Questo launch avvia una sola simulazione, camera e bridge, controllori, MoveIt,
planning scene, grasp manager, pipeline visiva e pick-and-place manager.
La sorgente statica è disabilitata in modalità `vision`.
Non avviare anche `camera_test.launch.py`, `vision_test.launch.py`,
`pear_pose_source.py` o un secondo pick-place manager.
La scena collisioni e le pose del cestino restano quelle dell'orchard attuale:
questa fase non implementa la randomizzazione del mondo.

## Controllo e trigger (terminale 2)

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 control list_controllers
ros2 topic info /harvest/pear_pose
ros2 topic echo /orchard/vision/center_status --once
ros2 topic echo /harvest/pear_pose --once
```

Attendere i controllori `active`, un solo publisher su `/harvest/pear_pose`,
stato `valid: true` e pose in `world` vicine al centro della pera.
Stato e posa letti con due comandi `--once` possono riferirsi a frame diversi;
il manager verifica la corrispondenza sul flusso ricevuto.

```bash
ros2 topic pub --once /harvest/trigger std_msgs/msg/Empty '{}'
```

Un rifiuto indica il motivo nel terminale 1. Se la posa è scaduta, attendere una
nuova rilevazione e riprovare. La soglia è in secondi; la frequenza bassa della
VM può rendere necessario attendere la misura successiva.
Inviare un solo trigger e attendere la conclusione del ciclo.
Verificare in Gazebo presa, distacco dal ramo, arretramento e deposito fisico.
Il preflight non certifica l'esecuzione completa in Gazebo.
Il limite di attesa delle traiettorie MoveIt e cartesiane è 120 s: nel test
precedente il trasporto aveva richiesto circa 67 s, superando il vecchio limite
di 60 s pur terminando con successo sul controller.
Nel terminale del manager controllare in ordine `[FASE 6/8]`,
`Movimento completato con successo`, `[FASE 7/8]`, `[FASE 8/8]` e
`SEQUENZA PICK AND PLACE COMPLETATA CON SUCCESSO`. Verificare anche il messaggio
`RELEASED` del grasp manager e la pera che cade nel cestino in Gazebo.
Se appare ancora un timeout, annotare sia l'ora del timeout del manager sia
l'eventuale successivo `Goal reached, success!` del controller: indicano se il
limite è ancora troppo breve rispetto alla velocità della simulazione.

## Visualizzazione opzionale (terminale 3)

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix rbt08pkg)/share/rbt08pkg/config/camera.rviz" --ros-args -p use_sim_time:=true
```

Per confrontare il target di presa, aggiungere un display `Pose` sul topic
`/harvest/grasp_pose`. Il centro arancione e il target devono coincidere prima
del movimento. Il marker della visione può continuare ad aggiornarsi durante
il ciclo; la sequenza usa la posa scelta al trigger.

## Modalità precedente

```bash
ros2 launch rbt10pkg_mit integrated_pick_place.launch.py pose_source:=static
```

È ancora il default. In questa modalità non occorre `center_status`.
Anche il manager standalone conserva `require_vision_status:=false` di default.
Per usarlo manualmente con la visione impostare
`--ros-args -p use_sim_time:=true -p require_vision_status:=true` e abilitare
`publish_harvest_pose:=true` sul launch della visione, senza publisher statico.
