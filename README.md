# InverseKinematicsRP

Implementazione in C++ di cinematica diretta e inversa per manipolatori descritti con parametri DH in YAML oppure con URDF. Il progetto include una libreria cinematica e un nodo ROS 2 che carica un modello all’avvio, legge un target cartesiano da YAML quando viene chiamato un service e stampa la configurazione calcolata.

La descrizione tecnica è in [Documentation.md](Documentation.md).

## Funzionalità presenti

- `DHModel` carica i parametri DH da YAML; `URDFModel` ricava la catena da un file URDF tramite `urdfdom`.
- Entrambi implementano `KinematicModel` e forniscono la Forward Kinematics.
- Il Jacobiano lineare è calcolato numericamente perturbando una coordinata alla volta.
- La pseudoinversa del Jacobiano usa la SVD di Eigen.
- La IK aggiorna iterativamente la configurazione e controlla la convergenza sulla posizione cartesiana dell’end-effector.
- Dopo ogni aggiornamento, gli angoli dei giunti rotativi sono normalizzati in `[-π, π]`; le coordinate prismatiche non vengono normalizzate.
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
| 6doF-flv | R-R-R-R-P-R (5R1P) | spaziale |

I file sono in src/manipulator_ik/Robots e si selezionano passando al nodo il nome del robot e il suffisso -DH o -URDF, senza estensione. I modelli Planar lavorano nel piano XY; SCARA lavora in XY con corsa verticale verso il basso; RRP-flv e 6doF-flv sono spaziali. In 6doF-flv, J1 ruota attorno a Z, J2 e J3 fanno il pitch, J4 ruota attorno all’asse del link precedente, J5 estende il telescopio di 0–0.25 m e J6 inclina il tool. In SCARA, J1 e J2 ruotano attorno a Z, J3 estende verso il basso la colonna telescopica da 0 a 0.25 m e J4 ruota attorno a Z. Il TCP (tool0) si trova alla punta dello stelo, con quota Z tra 0.25 e 0.50 m. Ogni modello dichiara i limiti dei propri giunti; l’IK applica i limiti dichiarati ai giunti prismatici.

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
```

Con il nodo attivo, da un secondo terminale richiamare il service:

```bash
cd ~/InverseKinematicsRP
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /EE_pos_update std_srvs/srv/Trigger "{}"
```

Ogni chiamata rilegge il file. Dopo aver modificato le coordinate, richiamare il service per ricalcolare la configurazione.

> **Nota sul modello planare:** i modelli 3R d’esempio hanno tutti gli assi di rotazione lungo z e mantengono l’end-effector nel piano `z = 0`. Un target con `z` diverso da zero non è raggiungibile. Il `targetPosition.yaml` presente nel repository al momento della documentazione riporta `z: 0.10`; impostarlo a `0.0` per una richiesta raggiungibile dal modello planare.

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

## Limiti attuali e lavoro rimanente

- La IK risolve la posizione dell’end-effector, non l’orientamento.
- Il nodo pubblica la configurazione su /joint_states quando l’IK converge.
- Restano da creare un launch file e una configurazione RViz per avviare i nodi insieme.
