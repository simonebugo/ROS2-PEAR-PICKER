# Segmentazione HSV della pera - collaudo nel container

Il percorso attivo usa OpenCV del sistema ROS, senza Ultralytics, PyTorch,
pesi o venv. Il detector seleziona regioni gialle, pulisce la maschera e sceglie
la componente piu' grande che supera i filtri geometrici.
Il detector esegue la detection 2D; il launch avvia anche la stima della
superficie 3D descritta in [depth_phase.md](depth_phase.md). Nessuno dei due
pubblica /harvest/pear_pose o avvia il pick-and-place.
La camera RGB-D e i TF esistenti sono riutilizzati.

## 1. Copia selettiva da Windows

Chiudere il precedente vision_test.launch.py con Ctrl+C e attendere che i
processi terminino. Controllare il container con docker ps e modificare
l'ID se necessario. In CMD oppure PowerShell Windows:

```text
docker ps
powershell -NoProfile -ExecutionPolicy Bypass -File "D:\Matteo\Desktop\sr_ws10\vision\copy_hsv_to_container.ps1" -ContainerId 5f8ebe78a7a6
```

Lo script controlla che il pacchetto esista in /root/sr_ws10/src/rbt08pkg e
copia solo sette file della visione nel pacchetto (detector, algoritmo, launch,
YAML, RViz, manifest e CMake) e cinque strumenti/documenti in vision.
Se Docker segnala un errore la copia si interrompe con un messaggio esplicito.

Si aggiornano solo questi file. Non copiare la root del workspace, i venv o
i dataset. World, robot, controller, gripper e MoveIt non cambiano in questa
migrazione. La pera deve essere quella gialla configurata nel passo precedente.

Il codice training e i requirements YOLO sono ora in vision/legacy_yolo.
I pesi in vision/models e gli ambienti virtuali sono conservati ma inutilizzati.
docker cp non elimina vecchi file: eventuali vision/train.py,
vision/requirements.txt e vision/pear_container.yaml gia' nel container sono
residui inattivi. Non eseguirli per questa pipeline; non occorre cancellarli
per eseguire HSV. Gli argomenti weights, device, confidence e yolo_python
del vecchio launch non esistono piu'.

## 2. Dipendenze, build e test automatici

Aprire un terminale del container fuori dal venv. Se il prompt mostra un
ambiente attivo, usare deactivate (o conda deactivate per Conda).
Non serve ricreare o disinstallare ambienti.

```bash
source /opt/ros/jazzy/setup.bash
apt-get update
apt-get install -y python3-opencv python3-numpy python3-yaml \
  ros-jazzy-cv-bridge ros-jazzy-vision-msgs
cd /root/sr_ws10
colcon build --packages-select rbt08pkg --symlink-install
source install/setup.bash
/usr/bin/python3 /root/sr_ws10/vision/test_segmentation.py -v
/usr/bin/python3 /root/sr_ws10/vision/check_runtime.py
```

Il primo comando di test deve terminare con **15 test, OK**. Copre colori del
frutto, ombre moderate, scene negative, rumore, buchi, scelta tra piu' componenti,
filtri area/forma, configurazioni errate e assenza di target residui.

Il secondo usa veri publisher/subscriber ROS su un namespace di prova isolato,
senza Gazebo. Verifica RGB8->BGR, bbox, maschere, timestamp e frame, parametro
non valido, frame negativo, interruzione dello stream e ripresa dopo errore.
Durante il test sono attesi un warning RGB fermo e un errore per un encoding
volutamente invalido. Il risultato finale deve essere **PREFLIGHT OK**.
Qualsiasi eccezione finale indica un test fallito: fermarsi e conservare il log.

Questi test non dimostrano ancora che le soglie siano adatte ai pixel effettivi
del rendering Gazebo: occorre anche il controllo visivo seguente.

## 3. Avvio della simulazione e detector

```bash
cd /root/sr_ws10
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rbt08pkg vision_test.launch.py render_engine:=ogre
```

Atteso: "Segmentazione HSV pronta" e "CAMERA READY".
Non avviare contemporaneamente camera_test.launch.py: e' incluso.
Il launch esegue il detector con /usr/bin/python3.
In RViz sono disponibili:

- RGB: immagine originale.
- HSV pear: box verde e centro bbox rosso; area e fill in sovrimpressione.
- HSV mask: tutte le regioni gialle dopo la pulizia morfologica.
- Target mask: solo la regione selezionata, bianca su nero.
- Depth: immagine delle distanze esistente.

Se una camera e' gia' avviata, aggiungere start_camera:=false. In questo caso
non vengono avviati ne' camera ne' RViz dal launch. Per ridurre il carico nel
test completo e' disponibile rviz:=false; Gazebo continua a renderizzare RGB-D.

## 4. Diagnostica in un secondo terminale

```bash
source /opt/ros/jazzy/setup.bash
source /root/sr_ws10/install/setup.bash
ros2 topic echo /orchard/camera/status --once
ros2 topic echo /orchard/vision/status
```

Interrompere echo con Ctrl+C, poi:

```bash
ros2 topic echo /orchard/vision/detections --once
ros2 topic hz /orchard/camera/rgb/image_raw
ros2 topic hz /orchard/vision/image_annotated
```

Eseguire i due hz separatamente, per almeno 30 secondi ciascuno.
Non aggiungere --qos-reliability a hz nella vostra versione ROS.
Le immagini e le maschere usano QoS Best Effort; detection/status usano Reliable.

Lo stato /orchard/vision/status contiene:

| Campo/stato | Significato |
| --- | --- |
| DETECTED | Una regione supera le soglie e i filtri. |
| NO_TARGET | Frame elaborato correttamente, nessuna regione valida. |
| WAITING_IMAGE | Il detector non ha ancora ricevuto RGB. |
| STREAM_TIMEOUT | Nessun RGB da oltre stream_timeout_s secondi reali. |
| ERROR | Conversione o elaborazione fallita; leggere error. |
| processing_ms | Tempo di elaborazione/pubblicazione dell'ultimo frame, non latenza totale. |
| rgb_age_s | Secondi reali dall'ultima immagine ricevuta dal nodo. |
| valid_candidates | Quante regioni superano i filtri prima della selezione. |

ready e' vero sia per DETECTED sia per NO_TARGET. Per un target servono anche
detected=true e frame recente. Il detector elabora al massimo 5 Hz, conservando
solo il messaggio RGB piu' recente. Non accelera il rendering di Gazebo.

Al timeout pubblica una detection vuota e maschere nere, conservando lo stamp
dell'ultima immagine (non inventa una nuova misura). Riprende con il prossimo RGB.
La stima 3D abbina timestamp esatti e invalida risultati senza target o scaduti.

## 5. Regolazione senza modificare il codice

Le soglie iniziali sono H=18..38, S=100..255, V=45..255. In OpenCV uint8
H va da 0 a 179; S e V da 0 a 255. I valori dipendono dalla resa visiva effettiva.

Prima osservare HSV mask. Se contiene la pera ma Target mask e' nera,
controllare i filtri geometrici; se anche HSV mask e' nera, controllare il colore.

Esempi di prove temporanee (non applicarli tutti automaticamente):

```bash
ros2 param get /pear_detector hsv_lower
ros2 param set /pear_detector hsv_lower "[18, 80, 35]"
ros2 param set /pear_detector min_area_px 40
ros2 param set /pear_detector min_fill_ratio 0.3
```

Una soglia H piu' bassa puo' includere il marrone di ramo/cestino; una piu'
alta puo' includere il verde. Non selezionare l'intero sfondo per inseguire il
frutto. I cambiamenti agiscono dal prossimo frame e si perdono al riavvio.
Per renderli permanenti modificare rbt08pkg/config/pear_segmentation.yaml
nei sorgenti locali e ricopiare/ricompilare. Il launch accetta anche
detector_config:=/percorso/pear_segmentation.yaml.

Se RGB arriva realmente meno di una volta ogni 10 secondi, lo stato puo'
alternare DETECTED e STREAM_TIMEOUT. Come prova si puo' impostare
stream_timeout_s a 30.0, ma questo non risolve la lentezza e consente target
piu' vecchi: non e' una taratura finale per il grasp.

## 6. Salvare un frame reale per la taratura

Con camera attiva:

```bash
/usr/bin/python3 /root/sr_ws10/vision/inspect_segmentation.py \
  --config /root/sr_ws10/install/rbt08pkg/share/rbt08pkg/config/pear_segmentation.yaml
```

Salva una nuova sottocartella con timestamp in vision/runs/hsv_debug:
rgb.png, mask.png, target_mask.png, annotated.png e report.json.
La configurazione e' quella del YAML, non eventuali ros2 param set temporanei.
I valori provati si possono passare con --hsv-lower e --hsv-upper.

Per ripetere la segmentazione sul frame salvato, senza Gazebo:

```bash
/usr/bin/python3 /root/sr_ws10/vision/inspect_segmentation.py \
  --image /percorso/stampato/rgb.png --hsv-lower 18 80 35 --hsv-upper 38 255 255
```

## 7. Criteri di accettazione e limiti

- La maschera selezionata copre il corpo/collo giallo della pera e non il robot.
- Bbox e centro seguono il frutto in piu' posizioni/distanze.
- Quando il frutto scompare, detection vuota e Target mask nera.
- Sfondi senza pera non producono falsi target.
- Una pausa prolungata della simulazione invalida il target; Play lo ripristina.
- Ombre moderate non eliminano tutto il frutto.

Il sistema riconosce una regione gialla plausibile, non identifica semanticamente
una pera. Un oggetto giallo simile puo' essere selezionato al suo posto; se ci sono
piu' candidati vince quello con area maggiore. Una regione visibile parzialmente
puo' spostare bbox e centro rispetto al frutto completo.

Detection2DArray e maschere mantengono frame e timestamp RGB originali.
bbox.center e' il centro geometrico della bounding box; il centroide della
maschera e' separato nel target del messaggio status.
class_id resta pear. **score e' area della regione / area della bbox**,
un indice geometrico [0,1], non la probabilita' YOLO: non confrontarlo con le
vecchie soglie confidence. Il picciolo marrone non fa parte della maschera.

La stima camera-frame campiona gia' la depth nei pixel interni della maschera.
Le fasi successive trasformeranno il punto con TF e definiranno la correzione
per il grasp prima di collegare la posa al pick-and-place.
Non usare ancora questo test come validazione del grasp guidato dalla camera.

Riferimenti: [sogliatura HSV OpenCV](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html),
[componenti connesse](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html).

## Stima 3D della superficie

La nuova fase camera-frame è descritta in [depth_phase.md](depth_phase.md),
con build, test automatici e verifiche RViz. Non è ancora collegata al robot.

## Trasformazione in world

Il launch avvia anche la trasformazione TF2 del punto di superficie:
[world_phase.md](world_phase.md) descrive preflight e confronto dei marker.
Non è ancora una posa di grasp e non avvia il pick-and-place.

## Centro stimato

Il transformer pubblica ora anche un centro approssimato e un marker arancione.
Formula, limiti e collaudo: [center_phase.md](center_phase.md).
