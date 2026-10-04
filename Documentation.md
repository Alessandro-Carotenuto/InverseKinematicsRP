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

Il test di equivalenza conferma che producono stessi DOF, limiti e FK.

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
          implementato      implementato
```

Il Jacobiano generico riceve `KinematicModel`. Non dipende più dalla rappresentazione DH.

### 5. Integrazione `urdfdom`

`urdfdom` legge l'XML e restituisce un `urdf::ModelInterface` contenente link, joint e relazioni parent/child.

```cpp
urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(path);
```

Il test di caricamento verifica entrambi gli URDF: nome, root, link, joint, tipi, assi, origini e limiti.

## Scelte di design

- `urdfdom` analizza l'XML. Il progetto non implementa un parser.
- Il progetto implementa FK, Jacobiano e IK. Non usa KDL per questi calcoli.
- `DHModel` rimane disponibile insieme a `URDFModel`.
- `KinematicModel` permette agli algoritmi di ignorare la rappresentazione interna.
- Codice cinematico indipendente dal futuro nodo ROS 2.
- `robot_state_publisher`, TF e RViz serviranno solo per trasformazioni ROS e visualizzazione.

## `URDFModel` e Forward Kinematics

Il modello viene costruito indicando file, base ed end-effector:

```cpp
URDFModel::FromFile(path, "base_link", "tool0");
```

La catena viene ricavata risalendo da `tool0` a `base_link`, poi invertita.

Ogni joint interno conserva:

- tipo: revolute, prismatic o fixed;
- trasformazione `origin`;
- `axis` normalizzato;
- limiti;
- indice dentro `q`, oppure `-1` per joint fixed.

La FK usa:

```text
T = T × Origin × JointMotion(q)
```

- revolute: rotazione attorno ad `axis`;
- prismatic: traslazione lungo `axis`;
- fixed: identità, senza elemento in `q`.

Il test verifica metadati, configurazione zero, entrambi i joint revolute, joint prismatic e dimensione di `q`.

Il Jacobiano generico funziona con `URDFModel`; il test numerico coincide con il Jacobiano analitico del manipolatore RRP.

## Flusso attuale

```text
DHModel oppure URDFModel
          ↓
    KinematicModel
          ↓
FK / Jacobiano / Inverse Kinematics
```

La vecchia API basata su `DHList` rimane disponibile tramite un wrapper che costruisce `DHModel`.

Il test diretto con `DHModel` conferma che la IK generica raggiunge i target cartesiani senza usare il wrapper.

Il test diretto con `URDFModel` conferma lo stesso flusso usando la catena caricata dal file URDF.
