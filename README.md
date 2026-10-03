# InverseKinematicsRP

## Documentazione

La struttura dei modelli cinematici e le scelte progettuali sono descritte in [Documentation.md](Documentation.md).

## Roadmap

- [x] 1. Denavit–Hartenberg
- [x] 2. Forward Kinematics
- [x] 3. Jacobiano numerico
- [x] 4. Pseudoinversa dello Jacobiano
- [x] 5. Inverse Kinematics iterativa
- [x] 6. Modello URDF del manipolatore RRP
- [x] 7. Interfaccia comune `KinematicModel`
- [x] 8. `DHModel` e Jacobiano numerico indipendente dal modello
- [x] 9. Integrazione di `urdfdom` per la lettura dei file URDF
- [x] 10. `URDFModel` e Forward Kinematics da `origin`, `axis` e tipo di joint
- [ ] 11. Test di equivalenza tra i modelli URDF cinematico e visuale
- [ ] 12. Verifica del Jacobiano generico con `URDFModel`
- [ ] 13. Adattamento della Inverse Kinematics a `KinematicModel`
- [ ] 14. Nodo ROS 2
- [ ] 15. Parametri YAML
- [ ] 16. JointState / robot_state_publisher / TF / RViz
- [ ] 17. Estensione oltre il mero traslazionale

La Inverse Kinematics attuale considera soltanto la posizione cartesiana dell'end-effector, non il suo orientamento.

## Scelte tecniche

- `urdfdom` viene usato per analizzare l'XML: non viene sviluppato un parser URDF interno.
- L'estrazione della catena e la Forward Kinematics da URDF vengono implementate nel progetto, senza KDL.
- Il Jacobiano numerico e la Inverse Kinematics rimangono implementazioni del progetto.
- `DHModel` e `URDFModel` condividono l'interfaccia `KinematicModel`, quindi la cinematica DH resta disponibile.
- Il nodo usa `rclcpp`; `robot_state_publisher`, TF e RViz gestiscono pubblicazione delle trasformazioni e visualizzazione.
- Gli URDF cinematico e visuale descrivono la stessa catena; il secondo aggiunge soltanto geometrie e materiali.
