# Stima del centro per verifica del grasp

Il transformer esistente calcola anche un centro approssimato in world:

    center = surface + radius * (surface - camera_origin) / norm(surface - camera_origin)

La superficie deriva dalla depth. L'origine camera deriva dalla stessa TF
usata per trasformare la misura, allo stesso timestamp. Il raggio configurabile
e' 0.075 m, corrispondente al corpo del modello attuale. Nessuna coordinata
nota del centro Gazebo viene usata dal nodo.

E' un'approssimazione: e' esatta per una sfera quando il raggio visivo attraversa
il centro. Un campione laterale, sul collo, una maschera parziale o un frutto di
dimensione diversa possono produrre bias. VALID significa calcolo disponibile,
non accuratezza di grasp garantita. Non e' stato aggiunto filtraggio temporale.

## Aggiornamento

Chiudere il vecchio launch. Da Windows nella root del progetto:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File vision\copy_hsv_to_container.ps1 -ContainerId 5f8ebe78a7a6
```

Nel container, senza venv:

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rbt08pkg --symlink-install
source install/setup.bash
/usr/bin/python3 vision/test_center.py -v
/usr/bin/python3 vision/check_world_runtime.py
ros2 launch rbt08pkg vision_test.launch.py render_engine:=ogre
```

Attesi: quattro test OK e WORLD PREFLIGHT OK (include ora centro stimato).
Il preflight usa topic privati e non richiede Gazebo.

## Verifica

In ogni nuovo terminale caricare prima i due setup.bash, quindi eseguire
separatamente:

```bash
ros2 topic echo /orchard/vision/center_status
ros2 topic echo /orchard/vision/center_point_world
```

Atteso: VALID, frame world, radius_m 0.075, timestamp originale della superficie.
In RViz, Fixed Frame world, il nuovo display Pear estimated center mostra una
sfera arancione di diametro 2.5 cm. Il ciano rimane sulla superficie; l'arancione
deve essere piu' interno, allontanato dalla camera di 7.5 cm.
Se necessario aggiungere Marker con topic /orchard/vision/center_marker_world.
Il centro puo' essere nascosto da una visualizzazione opaca del frutto.

Con pera ferma nella posa iniziale confrontare numericamente con (0.85, 0.55, 1.30).
Calcolare errore = sqrt((x-.85)^2 + (y-.55)^2 + (z-1.30)^2).
Raccogliere almeno 20 misure e controllare errore e oscillazioni: un primo
obiettivo di collaudo e' errore sotto 1.5 cm nella posa attuale, non una
garanzia sufficiente di presa. La tolleranza finale dipendera' dal gripper.
Se la pera si e' spostata, confrontare con la sua posa corrente in Gazebo.
La posa di riferimento va usata soltanto nella valutazione.

Verificare inoltre che dopo una perdita del target o piu' di 10 secondi reali
senza misure il centro sia invalido e il marker venga eliminato. Riprendendo
lo stream deve riapparire. Un vecchio PointStamped gia' ricevuto da un consumer
non viene cancellato: controllare anche stato e freschezza nelle fasi successive.

Parametri di launch: estimate_center:=false disabilita il centro;
fruit_radius_m:=0.075 imposta il raggio. Richiede estimate_3d e transform_world
attivi. Cambiare il raggio solo sulla base della geometria, non per compensare
errori di calibrazione.

Non viene pubblicato /harvest/pear_pose. Il manager attuale calcola gia'
l'orientamento del gripper dalla posizione: una posa dell'oggetto non equivale
all'orientamento del gripper. Dopo questa verifica restano stabilizzazione,
validazione della posa e collegamento al manager.
