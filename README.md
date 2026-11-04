# InverseKinematicsRP

Implementazione in C++ di cinematica diretta e inversa per manipolatori descritti con parametri DH in YAML oppure con URDF. Il progetto include una libreria cinematica e un nodo ROS 2 che carica un modello all’avvio, legge un target di posizione o posa da YAML quando viene chiamato un service e stampa la configurazione calcolata.

La descrizione tecnica è in [Documentation.md](Documentation.md).

## Funzionalità presenti

- `DHModel` carica i parametri DH da YAML; `URDFModel` ricava la catena da un file URDF tramite `urdfdom`.
- Entrambi implementano `KinematicModel` e forniscono la Forward Kinematics.
- I Jacobiani di posizione e orientamento sono calcolati numericamente perturbando una coordinata alla volta.
- La pseudoinversa del Jacobiano usa la SVD di Eigen.
- La IK aggiorna iterativamente la configurazione: `ONLY_POSITION` controlla XYZ, mentre `FULL_POSE` controlla posizione e orientamento.
- Dopo ogni aggiornamento, gli angoli rotativi vengono riportati in `[-π, π]` e tutti i giunti limitati vengono vincolati ai rispettivi limiti.
- Il nodo ROS 2 carica un modello DH o URDF all’avvio e offre il service `EE_pos_update`.
- Il service rilegge `targetPosition.yaml`, calcola la IK e restituisce lo stato tramite `std_srvs/srv/Trigger`; configurazione e convergenza sono anche stampate nei log.

## Modelli di esempio

Per ogni robot sono disponibili entrambe le descrizioni: parametri DH in YAML e catena URDF.

| Robot | Giunti | Piano |
|---|---|---|
| 2R-Planar | R-R | XY |
| 3R-Planar | R-R-R | XY |
| SCARA | R-R-P-R | XY + Z |
| RRP-flv | R-R-P | spaziale |
| Stanford | R-R-P-R-R-R | spaziale, posa completa |

I file sono in `src/manipulator_ik/Robots` e si selezionano passando al nodo il nome del robot e il suffisso `-DH` o `-URDF`, senza estensione. Il campo `robot.ik_mode` nel file DH dichiara `ONLY_POSITION` o `FULL_POSE`; il file URDF associato eredita la stessa modalità. 2R-Planar, 3R-Planar, SCARA e RRP-flv risolvono XYZ. Stanford è il modello classico R-R-P-R-R-R e risolve anche l’orientamento del TCP. In SCARA J1 e J2 ruotano attorno a Z, J3 estende verso il basso da 0 a 0.25 m e J4 ruota attorno a Z. Ogni modello dichiara i limiti dei propri giunti, applicati dall’IK.

## Build e avvio

Eseguire i comandi dalla root del workspace, così il nodo trova i percorsi relativi dei modelli e del file target:

```bash
cd ~/InverseKinematicsRP
source /opt/ros/jazzy/setup.bash
colcon build --packages-select manipulator_ik
source install/setup.bash
ros2 run manipulator_ik manipulator_ik_node 3R-Planar-DH
```

L’ultimo argomento è il nome del modello senza estensione. Per usare l’URDF, avviare invece con:

```bash
ros2 run manipulator_ik manipulator_ik_node 3R-Planar-URDF
```

Il nodo cerca il file corrispondente in `src/manipulator_ik/Robots`. Per l’URDF, il nodo usa i link `base_link` e `tool0`.

## Modificare il target e richiamare la IK

Il file letto dal nodo è `src/manipulator_ik/src/targetPosition.yaml`:

```yaml
position:
  x: 2.0
  y: 0.1
  z: 0.0
orientation:
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

Con il nodo attivo, da un secondo terminale richiamare il service:

```bash
cd ~/InverseKinematicsRP
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /EE_pos_update std_srvs/srv/Trigger "{}"
```

Ogni chiamata rilegge il file. Dopo aver modificato il target, richiamare il service per ricalcolare la configurazione. I modelli `ONLY_POSITION` usano soltanto `position`; per Stanford (`FULL_POSE`) `orientation` è obbligatoria e gli angoli RPY sono in radianti.

> **Nota sui modelli planari:** i modelli 3R d’esempio hanno tutti gli assi di rotazione lungo z e mantengono l’end-effector nel piano `z = 0`. Un target con `z` diverso da zero non è raggiungibile.

## Visualizzazione con RViz

Gli URDF contengono geometrie visuali: base ed end effector sono arancioni, i link intermedi bianchi e gli steli prismatici mobili blu. Le varianti DH in YAML non contengono geometrie visuali.

Se i pacchetti non sono installati:
~~~bash
sudo apt install ros-jazzy-rviz2 ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui
~~~

Per provare il modello URDF con slider manuali, usa tre terminali.

Terminale 1, dalla root del workspace:
~~~bash
source /opt/ros/jazzy/setup.bash
cd ~/InverseKinematicsRP
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/manipulator_ik/Robots/RRP-flv-URDF.urdf)"
~~~

Terminale 2:
~~~bash
source /opt/ros/jazzy/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
~~~

Terminale 3:
~~~bash
source /opt/ros/jazzy/setup.bash
rviz2
~~~

In RViz imposta Fixed Frame su base_link e aggiungi il display RobotModel. La GUI pubblica stati manuali dei giunti; robot_state_publisher li converte in trasformazioni TF.

Il nodo manipulator_ik_node pubblica la configurazione su /joint_states quando l’IK converge. Con robot_state_publisher attivo e la descrizione URDF aggiornata, RViz segue i nuovi stati. Usa joint_state_publisher_gui solo per la prova manuale, non insieme al publisher del nodo.

### Procedura per visualizzare la IK in RViz

Per usare la procedura interattiva, apri RViz in un terminale:

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

In un secondo terminale avvia `robot_state_publisher` con un URDF di esempio. Qui viene usato RRP-flv; il nodo IK sostituirà poi questa descrizione con quella del modello scelto:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/InverseKinematicsRP/src/manipulator_ik/Robots/RRP-flv-URDF.urdf)"
```

In RViz imposta `Fixed Frame` su `base_link`, aggiungi il display `RobotModel` e scegli `Topic` come sorgente della descrizione, con topic `/robot_description`.

In un terzo terminale carica l’ambiente ROS e il workspace, poi avvia il nodo con il modello desiderato:

```bash
source /opt/ros/jazzy/setup.bash
source ~/InverseKinematicsRP/install/setup.bash
ros2 run manipulator_ik manipulator_ik_node SCARA-URDF
```

Il nodo carica SCARA-URDF e aggiorna il parametro `robot_description` di `robot_state_publisher`; RViz mostra quindi il modello selezionato e gli stati articolari pubblicati su `/joint_states`.

Per calcolare la configurazione del target corrente, chiama il service da un altro terminale con ROS e workspace caricati:

```bash
source /opt/ros/jazzy/setup.bash
source ~/InverseKinematicsRP/install/setup.bash
ros2 service call /EE_pos_update std_srvs/srv/Trigger "{}"
```

Il service rilegge `src/manipulator_ik/src/targetPosition.yaml` a ogni chiamata: puoi modificare il target e richiamarlo tutte le volte che vuoi. Per cambiare modello senza riavviare il nodo, usa il parametro `model_name`, ad esempio:

```bash
ros2 param set /manipulator_ik model_name RRP-flv-URDF
```

Il nodo aggiorna anche la descrizione URDF in `robot_state_publisher`; puoi poi richiamare il service per calcolare la IK sul nuovo modello.

## Limiti attuali e lavoro rimanente

- 2R-Planar, 3R-Planar, SCARA e RRP-flv risolvono la posizione; Stanford risolve la posa completa entro il proprio spazio di lavoro.
- Il nodo pubblica la configurazione su /joint_states quando l’IK converge.
- Restano da creare un launch file e una configurazione RViz per avviare i nodi insieme.
- Rivedere codice per `NaiveSeed()`
