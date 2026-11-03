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

Il guess iniziale è zero per i giunti senza limiti; per i giunti limitati è il punto medio tra minimo e massimo dichiarati. Dopo ogni aggiornamento, le coordinate revolute vengono riportate con `std::remainder(q, 2π)` nell’intervallo angolare canonico `[-π, π]`; le coordinate prismatiche non vengono alterate.

La flag `Converged` è determinata dall’errore finale sulla posizione rispetto alla tolleranza. La routine non risolve l’orientamento dell’end-effector e non vincola gli aggiornamenti ai limiti meccanici del modello. La normalizzazione periodica degli angoli non equivale a imporre limiti reali.

## Nodo ROS 2

Il nodo si chiama `manipulator_ik_node` e riceve all’avvio il nome del file modello senza estensione. Cerca il file nella directory `src/manipulator_ik/Robots`; l’estensione `.yaml` seleziona `DHModel`, `.urdf` seleziona `URDFModel`.

Il service `EE_pos_update` usa `std_srvs/srv/Trigger`, quindi non riceve coordinate nella richiesta. Alla chiamata:

1. legge `position.x`, `position.y` e `position.z` da `src/manipulator_ik/src/targetPosition.yaml`;
2. calcola la IK sul modello caricato all’avvio;
3. imposta `success` e `message` nella risposta del service;
4. stampa convergenza e configurazione articolare nei log.

Il file YAML viene riletto a ogni chiamata: si può cambiare il target senza riavviare il nodo. Il nodo attuale non pubblica la configurazione su un topic e non pubblica `JointState`.

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

## Modelli presenti e attività rimanenti

I modelli correnti sono `3R-Planar-DH.yaml` e `3R-Planar-URDF.urdf`. Entrambi rappresentano tre giunti revolute planari e tre link di lunghezza unitaria. I file dichiarano limiti `[-π, π]`, inseriti come valori del modello d’esempio; il solver non li applica come vincoli durante l’iterazione.

Per il 3R planare la coordinata z dell’end-effector è sempre zero. Il file target presente in repository contiene attualmente `z: 0.10`, che non è raggiungibile con questi due modelli; usare `z: 0.0` per una richiesta planare raggiungibile.

Le attività che restano per lo scopo attuale sono:

- aggiungere altri modelli di robot in formato DH e/o URDF;
- aggiungere la visualizzazione del manipolatore, includendo la pubblicazione dello stato dei giunti e l’integrazione con TF/RViz.
