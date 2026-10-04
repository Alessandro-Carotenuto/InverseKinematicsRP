# InverseKinematicsRP

## Documentazione

La struttura dei modelli cinematici e le scelte progettuali sono descritte in [Documentation.md](Documentation.md).

## Roadmap

- [x] 1. Modello Denavit–Hartenberg
- [x] 2. Forward Kinematics DH
- [x] 3. Jacobiano numerico
- [x] 4. Pseudoinversa tramite SVD
- [x] 5. Inverse Kinematics iterativa
- [x] 6. URDF cinematico del manipolatore RRP
- [x] 7. URDF visuale del manipolatore RRP
- [x] 8. Interfaccia comune `KinematicModel`
- [x] 9. `DHModel : KinematicModel`
- [x] 10. Jacobiano generico basato su `KinematicModel`
- [x] 11. Integrazione della libreria `urdfdom`
- [x] 12. Test di caricamento URDF
- [x] 13. `URDFModel : KinematicModel`
- [x] 14. Estrazione catena `base_link`–`tool0`
- [x] 15. Conversione `origin` URDF in matrice `4×4`
- [x] 16. Movimento joint revolute
- [x] 17. Movimento joint prismatic
- [x] 18. Gestione joint fixed
- [x] 19. Forward Kinematics URDF
- [x] 20. Test FK di `URDFModel`
- [x] 21. Equivalenza tra URDF cinematico e visuale
- [x] 22. Test del Jacobiano generico con `URDFModel`
- [x] 23. Adattamento della IK a `KinematicModel`
- [x] 24. Test IK con `DHModel`
- [x] 25. Test IK con `URDFModel`
- [ ] 26. Nodo ROS 2
- [ ] 27. Ricezione target cartesiano
- [ ] 28. Pubblicazione `JointState`
- [ ] 29. Parametri YAML
- [ ] 30. `robot_state_publisher` e TF
- [ ] 31. Configurazione RViz
- [ ] 32. Dimostrazione completa
- [ ] 33. Estensione IK all'orientamento dell'end-effector

La Inverse Kinematics attuale considera soltanto la posizione cartesiana dell'end-effector, non il suo orientamento.

## Scelte tecniche

- `urdfdom` viene usato per analizzare l'XML: non viene sviluppato un parser URDF interno.
- L'estrazione della catena e la Forward Kinematics da URDF vengono implementate nel progetto, senza KDL.
- Il Jacobiano numerico e la Inverse Kinematics rimangono implementazioni del progetto.
- `DHModel` e `URDFModel` condividono l'interfaccia `KinematicModel`, quindi la cinematica DH resta disponibile.
- Il nodo usa `rclcpp`; `robot_state_publisher`, TF e RViz gestiscono pubblicazione delle trasformazioni e visualizzazione.
- Gli URDF cinematico e visuale descrivono la stessa catena; il secondo aggiunge soltanto geometrie e materiali.
