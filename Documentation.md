# Documentazione tecnica

## Evoluzione del progetto

### 1. Modello Denavit–Hartenberg

`DHMatrix` rappresenta un joint tramite parametri DH, tipo, limiti e matrice omogenea `4×4`.

`ForwardKinematics(DHList)` moltiplica le matrici della catena e restituisce la posa dell'end-effector.

### 2. Jacobiano e Inverse Kinematics

Il Jacobiano lineare viene calcolato numericamente perturbando un joint alla volta.

La pseudoinversa usa SVD. La Inverse Kinematics aggiorna iterativamente `q` per ridurre l'errore cartesiano.

La IK attuale controlla solo la posizione, non l'orientamento.

### 3. Modello URDF

Sono presenti due descrizioni dello stesso manipolatore RRP:

- `manipulator_rrp.urdf`: sola cinematica;
- `manipulator_rrp_visual.urdf`: stessa cinematica con geometrie e materiali.

### 4. Interfaccia comune

`KinematicModel` definisce:

```cpp
ForwardKinematics(q)
getDOF()
getJointMin(index)
getJointMax(index)
isJointLimited(index)
```

`DHModel` implementa questa interfaccia usando la `DHList` esistente.

```text
                 KinematicModel
                 /            \
            DHModel          URDFModel
          implementato       pianificato
```

Il Jacobiano generico riceve `KinematicModel`. Non dipende più dalla rappresentazione DH.

### 5. Integrazione `urdfdom`

`urdfdom` legge l'XML e restituisce un `urdf::ModelInterface` contenente link, joint e relazioni parent/child.

```cpp
urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);
```

Il test attuale verifica entrambi gli URDF: nome, root, link, joint, tipi, assi, origini e limiti.

## Scelte di design

- `urdfdom` analizza l'XML. Il progetto non implementa un parser.
- Il progetto implementa FK, Jacobiano e IK. Non usa KDL per questi calcoli.
- `DHModel` rimane disponibile anche dopo l'aggiunta di URDF.
- `KinematicModel` permette agli algoritmi di ignorare la rappresentazione interna.
- Codice cinematico indipendente dal futuro nodo ROS 2.
- `robot_state_publisher`, TF e RViz serviranno solo per trasformazioni ROS e visualizzazione.

## Prossima implementazione: `URDFModel`

`URDFModel` caricherà una catena tra base ed end-effector.

Ogni joint interno conserverà:

- tipo: revolute, prismatic o fixed;
- trasformazione `origin`;
- `axis`;
- limiti;
- indice dentro `q`, oppure `-1` per joint fixed.

La catena verrà ricavata risalendo da `tool0` a `base_link`, poi invertita.

La FK userà:

```text
T = T × Origin × JointMotion(q)
```

- revolute: rotazione attorno ad `axis`;
- prismatic: traslazione lungo `axis`;
- fixed: identità, senza elemento in `q`.

Il Jacobiano generico funzionerà senza un secondo algoritmo.

## Flusso finale previsto

```text
DHModel oppure URDFModel
          ↓
    KinematicModel
          ↓
FK / Jacobiano / Inverse Kinematics
          ↓
          q
```
