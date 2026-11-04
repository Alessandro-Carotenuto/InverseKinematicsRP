# Documentazione tecnica

## Panoramica

Il repository contiene una libreria di cinematica e un nodo ROS 2. I due formati di modello convergono sulla stessa interfaccia:

```text
DH YAML ──> DHModel ───┐
                      ├──> KinematicModel ──> FK / Jacobiano / IK
URDF ──> urdfdom ─> URDFModel ─┘

targetPosition.yaml ──> service ROS ──> IK ──> stato e giunti nei log
```

## Struttura del codice

- `src/manipulator_ik/include/manipulator_ik/forward_kinematics.hpp`: parametri DH, `DHMatrix` e API delle FK/Jacobiane DH.
- `src/manipulator_ik/src/forward_kinematics.cpp`: matrice DH, FK DH e Jacobiano numerico.
- `src/manipulator_ik/include/manipulator_ik/kinematic_model.hpp`: interfaccia comune dei modelli.
- `src/manipulator_ik/src/dh_model.cpp`: modello DH e caricamento YAML.
- `src/manipulator_ik/src/urdf_model.cpp`: caricamento URDF, costruzione della catena e FK URDF.
- `src/manipulator_ik/src/inverse_kinematics.cpp`: pseudoinversa SVD e IK iterativa generica.
- `src/manipulator_ik/src/manipulator_ik_node.cpp`: nodo ROS 2, caricamento modello, lettura target e service.
- `src/manipulator_ik/Robots/`: modelli di esempio.

## Interfaccia dei modelli

`KinematicModel` espone le operazioni usate dagli algoritmi indipendentemente dal formato del modello:

- `ForwardKinematics(q)` restituisce una trasformazione omogenea `4×4`.
- `getDOF()` restituisce il numero di giunti mobili.
- `isJointRevolute(index)` distingue coordinate angolari e prismatiche.
- `getJointName(index)` restituisce il nome del giunto associato alla coordinata.
- `getJointMin(index)`, `getJointMax(index)` e `isJointLimited(index)` espongono i limiti dichiarati nel modello.

I giunti fissi URDF partecipano alla catena di trasformazioni, ma non hanno una coordinata in `q`.

## Modello DH da YAML

`DHModel::FromYAML(path)` usa `yaml-cpp` per leggere la sequenza `robot.joints`. Ogni giunto mobile dichiara `name`, `type`, `alpha`, `a`, `theta` e `d`; i limiti sono dichiarati con `limited`, `lower` e `upper`.

Per il formato caricato da YAML, la configurazione `q` si somma all’offset DH:

```text
revolute:  theta_effettivo = theta_yaml + q
prismatic: d_effettivo     = d_yaml + q
```

La FK costruisce le matrici DH configurate e le moltiplica nell’ordine della catena.

Il campo facoltativo robot.base.rpy specifica roll, pitch e yaw in radianti e viene applicato prima della catena DH. Nei modelli con suffisso Planar e giunti prismatici, robot.base.rpy orienta la base in modo che anche il primo asse P giaccia nel piano XY.

## Modello URDF

`URDFModel::FromFile(path, base_link, tool_link)` usa `urdfdom` per analizzare il file XML. Il parser risale dal tool verso la base, raccoglie i giunti parent e poi inverte la sequenza per ottenere l’ordine base-tool.

Per ogni passaggio della catena, la FK compone:

```text
T = T × origin × joint_motion(q)
```

- `origin` è la posa fissa dichiarata dal joint URDF.
- Un giunto revolute applica una rotazione attorno all’asse del giunto.
- Un giunto prismatic applica una traslazione lungo l’asse.
- Un giunto fixed applica solo `origin` e non usa una coordinata in `q`.

Il nodo attuale carica la catena URDF tra `base_link` e `tool0`.

## Jacobiano e Inverse Kinematics

Il Jacobiano lineare è calcolato con differenze finite: si calcola la posizione di base e si perturba una coordinata di `q` per volta. Ogni colonna misura come cambia la posizione dell’end-effector quando cambia quel giunto.

La pseudoinversa è costruita dalla decomposizione SVD di Eigen, azzerando i valori singolari sotto la soglia specificata. La IK usa l’aggiornamento iterativo `q ← q + J⁺ e`, dove `e` è la differenza tra posizione target e posizione corrente.

Il guess iniziale è zero per i giunti senza limiti; per i giunti limitati è il punto medio tra minimo e massimo dichiarati. Dopo ogni aggiornamento, gli angoli revolute sono normalizzati in [-π, π]; le coordinate prismatiche limitate vengono vincolate ai limiti dichiarati.

La flag Converged è determinata dall’errore finale sulla posizione rispetto alla tolleranza. La routine non risolve l’orientamento dell’end-effector; i limiti prismatici dichiarati sono applicati durante gli aggiornamenti.

## Nodo ROS 2

Il nodo si chiama `manipulator_ik_node` e riceve all’avvio il nome del file modello senza estensione. Cerca il file nella directory `src/manipulator_ik/Robots`; l’estensione `.yaml` seleziona `DHModel`, `.urdf` seleziona `URDFModel`.

Il service `EE_pos_update` usa `std_srvs/srv/Trigger`, quindi non riceve coordinate nella richiesta. Alla chiamata:

1. legge `position.x`, `position.y` e `position.z` da `src/manipulator_ik/src/targetPosition.yaml`;
2. calcola la IK sul modello caricato all’avvio;
3. imposta `success` e `message` nella risposta del service;
4. stampa convergenza e configurazione articolare nei log; se l’IK converge, pubblica anche sensor_msgs/msg/JointState su /joint_states.

Il file YAML viene riletto a ogni chiamata: si può cambiare il target senza riavviare il nodo. Se l’IK converge, il nodo pubblica la configurazione su /joint_states.

### Build e uso

Dalla root del workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select manipulator_ik
source install/setup.bash
ros2 run manipulator_ik manipulator_ik_node 3R-Planar-DH
```

Da un secondo terminale, dopo aver sourced ROS e `install/setup.bash`, chiamare:

```bash
ros2 service call /EE_pos_update std_srvs/srv/Trigger "{}"
```

I percorsi del nodo sono relativi alla directory di lavoro: avviare il processo dalla root del workspace.

## Visualizzazione in RViz

La catena di visualizzazione è:

~~~text
URDF con geometrie visuali + /joint_states
                 |
                 v
       robot_state_publisher
                 |
              /tf, /tf_static
                 |
                 v
          RViz RobotModel
~~~

Gli URDF in Robots includono geometrie visuali. Base ed end effector sono arancioni, i link intermedi bianchi e gli steli prismatici mobili blu. RViz usa la descrizione URDF e le trasformazioni TF per disegnare il robot.

Per una prova manuale:

1. Avviare robot_state_publisher con il parametro robot_description caricato dal file URDF.
2. Avviare joint_state_publisher_gui: legge la descrizione e pubblica sensor_msgs/msg/JointState su /joint_states con i valori degli slider.
3. Avviare rviz2, impostare Fixed Frame a base_link e aggiungere RobotModel.

Comandi di esempio, dalla root del workspace, usando RRP-flv:
~~~bash
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/manipulator_ik/Robots/RRP-flv-URDF.urdf)"
~~~

In altri terminali:
~~~bash
source /opt/ros/jazzy/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
~~~

~~~bash
source /opt/ros/jazzy/setup.bash
rviz2
~~~

Il service EE_pos_update pubblica la configurazione convergente su /joint_states. robot_state_publisher usa il messaggio per aggiornare TF e RViz aggiorna il display RobotModel. Non usare insieme joint_state_publisher_gui, che pubblicherebbe altri valori sullo stesso topic.

La sequenza può poi essere raccolta in un launch file con i parametri robot_description, il nodo IK, robot_state_publisher e RViz.

## Modelli presenti

Ogni robot è disponibile come file YAML DH e come URDF nella directory src/manipulator_ik/Robots. Il suffisso -DH seleziona il file YAML; -URDF seleziona l’URDF. Le catene URDF usano base_link e tool0, come richiesto dal nodo.

| Robot | Sequenza | Piano |
|---|---|---|
| 2R-Planar | R-R | XY |
| 3R-Planar | R-R-R | XY |
| SCARA | R-R-P-R | XY + Z |
| RRP-flv | R-R-P | spaziale |
| 6doF-flv | R-R-R-R-P-R (5R1P) | spaziale |

I giunti revolute hanno limiti d’esempio [-π, π]. I limiti prismatici sono specifici di ciascun file; in 6doF-flv il quinto giunto ha una corsa da 0 a 0.25 m. La IK limita le coordinate prismatiche ai limiti dichiarati. SCARA ha due assi revolute verticali; il giunto prismatico J3 ha corsa verso il basso 0–0.25 m e J4 ruota attorno a Z; il TCP (tool0) si trova alla punta dello stelo, con quota Z tra 0.25 e 0.50 m. RRP-flv e 6doF-flv sono modelli spaziali.

Restano da aggiungere una configurazione RViz e un launch file per avviare i nodi insieme.
