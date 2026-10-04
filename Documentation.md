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
getJointName(index)
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

### 6. Caricamento DH da YAML

`DHModel::FromYAML(path)` usa `yaml-cpp` per creare una `DHList` da file.

Ogni joint contiene nome, tipo, `alpha`, `a`, `theta`, `d` e limiti. Sono supportati joint `revolute` e `prismatic`.

I parametri variabili sono applicati come offset:

```text
revolute:  theta_effettivo = theta_yaml + q
prismatic: d_effettivo     = d_yaml + q
```

Il costruttore storico `DHModel(DHList)` conserva il comportamento precedente usando offset nulli. I nomi caricati sono esposti da `KinematicModel` per la futura pubblicazione ROS.

## Scelte di design

- `urdfdom` analizza l'XML. Il progetto non implementa un parser.
- `yaml-cpp` analizza lo YAML DH. Il progetto valida schema, tipi e limiti.
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

`getJointName(index)` restituisce i joint mobili, in ordine di configurazione `q`; i joint fixed non occupano un indice.

## Flusso attuale

```text
YAML DH → DHModel ─┐
                   ├→ KinematicModel → FK / Jacobiano / IK
URDF → URDFModel ──┘
```

La vecchia API basata su `DHList` rimane disponibile tramite un wrapper che costruisce `DHModel`.

Il test diretto con `DHModel` conferma che la IK generica raggiunge i target cartesiani senza usare il wrapper.

Il test diretto con `URDFModel` conferma lo stesso flusso usando la catena caricata dal file URDF.

## Reload dinamico futuro

Il primo nodo ROS 2 caricherà il modello nel costruttore e richiederà un riavvio per cambiarlo.

Una futura estensione permetterà di cambiare modello mentre il nodo rimane attivo. Un service ROS riceverà la richiesta di reload; un topic non verrà usato perché il reload è un comando con esito, non un flusso continuo di dati.

Il nuovo modello verrà costruito e validato in una variabile temporanea. `model_` verrà sostituito soltanto dopo un caricamento completo. Se caricamento o validazione falliscono, il nodo risponderà con un errore e continuerà a usare il modello precedente.

Un eventuale reset dello stato della IK verrà esposto tramite un service separato, quando il nodo introdurrà uno stato persistente da azzerare.
