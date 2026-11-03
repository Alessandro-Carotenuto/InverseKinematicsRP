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

| File | Formato | Descrizione |
|---|---|---|
| `src/manipulator_ik/Robots/3R-Planar-DH.yaml` | YAML / DH | 3 giunti rotativi planari, link di lunghezza 1 |
| `src/manipulator_ik/Robots/3R-Planar-URDF.urdf` | URDF | La stessa catena planare 3R, con base `base_link` e tool `tool0` |

Entrambi i modelli d’esempio dichiarano limiti angolari `[-π, π]`. La IK usa i limiti per scegliere il guess iniziale, ma non vincola ogni aggiornamento entro i limiti dichiarati: la normalizzazione degli angoli non sostituisce un algoritmo IK vincolato.

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

## Limiti attuali e lavoro rimanente

- La IK risolve la posizione dell’end-effector, non l’orientamento.
- Il nodo stampa la configurazione, ma non pubblica `sensor_msgs/msg/JointState`.
- Restano da aggiungere altri modelli di robot e la visualizzazione del manipolatore.
