# Documentazione tecnica

## Panoramica

Il repository contiene una libreria di cinematica e un nodo ROS 2. I due formati di modello convergono sulla stessa interfaccia:

```text
DH YAML ──> DHModel ───┐
                      ├──> KinematicModel ──> FK / Jacobiano / IK
URDF ──> urdfdom ─> URDFModel ─┘

targetPosition.yaml (XYZ/RPY) ──> service ROS ──> IK ──> stato e giunti nei log
```

## Struttura del codice

- `src/manipulator_ik/include/manipulator_ik/forward_kinematics.hpp`: parametri DH, `DHMatrix` e API delle FK/Jacobiane DH.
- `src/manipulator_ik/src/forward_kinematics.cpp`: matrice DH e FK DH.
- `src/manipulator_ik/src/kinematic_model.cpp`: Jacobiani numerici di posizione e posa.
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
- `getIKTaskMode()` restituisce `IKTaskMode::ONLY_POSITION` o `IKTaskMode::FULL_POSE`.

I giunti fissi URDF partecipano alla catena di trasformazioni, ma non hanno una coordinata in `q`.

## Modello DH da YAML

`DHModel::FromYAML(path)` usa `yaml-cpp` per leggere la sequenza `robot.joints`. Ogni giunto mobile dichiara `name`, `type`, `alpha`, `a`, `theta` e `d`; i limiti sono dichiarati con `limited`, `lower` e `upper`. Il campo `robot.ik_mode` dichiara la modalità IK: `ONLY_POSITION` o `FULL_POSE`. Se il campo manca, il modello usa `ONLY_POSITION`. Il blocco facoltativo `robot.tool` applica una trasformazione fissa dopo la catena DH tramite `xyz` e `rpy`; entrambi usano metri e radianti e, se omessi, valgono zero.

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

Il nodo carica la catena URDF tra `base_link` e `tool0`. Per i modelli URDF abbina il file DH con suffisso `-DH.yaml` e ne usa `robot.ik_mode`, così la modalità resta dichiarata una sola volta per modello.

## Jacobiano e Inverse Kinematics

`ONLY_POSITION` usa un Jacobiano lineare 3×N calcolato con differenze finite. `FULL_POSE` usa un Jacobiano 6×N: le prime tre righe descrivono la velocità lineare e le ultime tre la velocità angolare, stimate perturbando ciascun giunto.

La pseudoinversa è costruita dalla decomposizione SVD di Eigen, azzerando i valori singolari sotto la soglia specificata. L’errore di orientamento è il vettore di rotazione minimo tra posa corrente e target. Per combinare metri e radianti, le componenti angolari del Jacobiano e dell’errore sono scalate con 0.25 m/rad. La convergenza richiede errore di posizione sotto `tol`; in `FULL_POSE` richiede anche errore angolare sotto 1e-3 rad.

Il guess iniziale è zero per i giunti senza limiti; per i giunti limitati è il punto medio tra minimo e massimo dichiarati. NaiveSeed() usa una ricerca coordinata con passi progressivamente dimezzati: in ONLY_POSITION minimizza l’errore XYZ; in FULL_POSE minimizza l’errore combinato di posizione e orientamento, scalando i radianti con 0.25 m/rad. Se il primo tentativo FULL_POSE non converge, l’IK prova quattro seed alternativi dei tre giunti rotativi finali. Dopo ogni aggiornamento, gli angoli revolute vengono riportati nell’intervallo [-π, π] e tutti i giunti limitati vengono vincolati ai limiti dichiarati.

## Nodo ROS 2

Il nodo si chiama `manipulator_ik_node` e riceve all’avvio il nome del file modello senza estensione. Cerca il file nella directory `src/manipulator_ik/Robots`; l’estensione `.yaml` seleziona `DHModel`, `.urdf` seleziona `URDFModel`.

Il service `EE_pos_update` usa `std_srvs/srv/Trigger`, quindi non riceve coordinate nella richiesta. Alla chiamata:

1. legge `position.x`, `position.y` e `position.z` da `src/manipulator_ik/src/targetPosition.yaml`;
2. se `ik_mode` è `FULL_POSE`, legge anche `orientation.roll`, `orientation.pitch` e `orientation.yaw` in radianti;
3. calcola la IK sul modello caricato;
4. imposta `success` e `message` nella risposta del service;
5. stampa convergenza e configurazione articolare nei log; se l’IK converge, pubblica anche sensor_msgs/msg/JointState su /joint_states.

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
| Stanford | R-R-P-R-R-R | spaziale, posa completa |

2R-Planar, 3R-Planar, SCARA e RRP-flv dichiarano `ONLY_POSITION`; Stanford dichiara `FULL_POSE`. I giunti revolute hanno limiti d’esempio [-π, π], salvo i limiti specifici Stanford. I limiti prismatici sono definiti nei file DH e URDF e applicati dall’IK. SCARA ha due assi revolute verticali; il giunto prismatico J3 ha corsa verso il basso 0–0.25 m e J4 ruota attorno a Z; il TCP (tool0) si trova alla punta dello stelo, con quota Z tra 0.25 e 0.50 m. RRP-flv e Stanford sono modelli spaziali.

Restano da aggiungere una configurazione RViz e un launch file per avviare i nodi insieme.


## Modello Stanford classico

Il manipolatore Stanford/Scheinman classico ha sei giunti nella sequenza R-R-P-R-R-R: due giunti rotativi alla base/spalla, un giunto prismatico telescopico e tre giunti rotativi al polso. Questa struttura è descritta nel [paper originale di Dobrotin e Scheinman (IJCAI 1973)](https://citeseerx.ist.psu.edu/document?doi=a011f5c232675569b1ec44516d1abff8338e5102&repid=rep1&type=pdf). La tabella seguente adotta la convenzione DH standard del modello Stanford nel [Robotics Toolbox di Peter Corke](https://raw.githubusercontent.com/petercorke/robotics-toolbox-matlab/master/models/mdl_stanford.m), che cita i dati cinematici da Richard Paul, [AIM-177, figura 2.3](https://github.com/petercorke/writings/releases/download/assets/aim-177.pdf).

| Giunto | Tipo | θ | d (m) | a (m) | α |
|---|---|---|---:|---:|---:|
| 1 | R | q₁ | 0.412 | 0 | −π/2 |
| 2 | R | q₂ | 0.154 | 0 | +π/2 |
| 3 | P | −π/2 | q₃ | 0 | 0 |
| 4 | R | q₄ | 0 | 0 | −π/2 |
| 5 | R | q₅ | 0 | 0 | +π/2 |
| 6 | R | q₆ | 0 | 0 | 0 |

Il tool ha un offset fisso di 0.263 m lungo l’asse z dopo il sesto giunto. Il modello di riferimento riporta per il prismatico J3 una corsa stimata di 12–50 in (0.3048–1.27 m); i limiti rotativi sono ±170° per J1, J2, J4 e J6 e ±90° per J5. Le quote sono quelle del modello cinematico pubblicato, non una calibrazione di un esemplare fisico. DH e URDF del repository devono rappresentare la stessa catena e lo stesso offset del tool.
