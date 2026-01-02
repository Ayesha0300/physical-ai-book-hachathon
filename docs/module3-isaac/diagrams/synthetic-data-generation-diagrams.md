# Diagrams for Synthetic Data Generation Concepts

## Figure SDG-001: Synthetic vs Real-World Data Collection Comparison

```
┌─────────────────────────────────────────────────────────────────┐
│              SYNTHETIC vs REAL-WORLD DATA COLLECTION           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  REAL-WORLD DATA COLLECTION          SYNTHETIC DATA GENERATION │
│  ┌─────────────────────────┐    ┌─────────────────────────┐   │
│  │ • Limited by physics    │    │ • Unlimited scalability │   │
│  │ • High cost & time      │    │ • Zero operational cost │   │
│  │ • Safety constraints    │    │ • Safe environment      │   │
│  │ • Manual annotation     │    │ • Automatic labeling    │   │
│  │ • Weather dependent     │    │ • Controlled conditions │   │
│  │ • Rare scenarios = hard │    │ • Generate any scenario │   │
│  │ • Quality varies        │    │ • Consistent quality    │   │
│  │ • Repeatability = low   │    │ • Exact repeatability   │   │
│  └─────────────────────────┘    └─────────────────────────┘   │
│                                                                 │
│  CHALLENGES:                        ADVANTAGES:               │
│  • Time & resource intensive        • Perfect ground truth    │
│  • Safety risks                     • Infinite data variety   │
│  • Annotation errors                • Domain randomization    │
│  • Environmental constraints        • Multi-modal sync        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram compares the limitations of real-world data collection with the advantages of synthetic data generation, highlighting key differences in scalability, cost, safety, and quality.

## Figure SDG-002: Synthetic Data Generation Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                SYNTHETIC DATA GENERATION PIPELINE               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │  Scene      │    │  Parameter  │    │  Data       │        │
│  │  Setup      │───►│  Variation  │───►│  Capture    │        │
│  │             │    │             │    │             │        │
│  │ • Objects   │    │ • Lighting  │    │ • RGB       │        │
│  │ • Lighting  │    │ • Textures  │    │ • Depth     │        │
│  │ • Cameras   │    │ • Poses     │    │ • Seg       │        │
│  │ • Sensors   │    │ • Weather   │    │ • Normals   │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│            │              │                     │              │
│            v              v                     v              │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              GROUND TRUTH GENERATION                    │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐      │   │
│  │  │  Semantic   │ │  Instance   │ │  Depth      │      │   │
│  │  │  Segmentation│ │  Segmentation│ │  Maps       │      │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘      │   │
│  │         │               │               │              │   │
│  │         v               v               v              │   │
│  │  ┌─────────────────────────────────────────────────┐   │   │
│  │  │              DATASET ASSEMBLY                   │   │   │
│  │  │  • Organized data structure                     │   │   │
│  │  │  • Metadata and annotations                     │   │   │
│  │  │  • Quality validation                           │   │   │
│  │  └─────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates the complete synthetic data generation pipeline, from scene setup through parameter variation, data capture, ground truth generation, and dataset assembly.

## Figure SDG-003: Domain Randomization Process

```
┌─────────────────────────────────────────────────────────────────┐
│                DOMAIN RANDOMIZATION PROCESS                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  BASE SCENE ────────────────────────────→ RANDOMIZED SCENES    │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐│
│  │   Kitchen   │ │   Kitchen   │ │   Kitchen   │ │   Kitchen   ││
│  │             │ │             │ │             │ │             ││
│  │ • Blue walls│ │ • Red walls │ │ • Green     │ │ • Gray      ││
│  │ • Wood floor│ │ • Tile floor│ │ • Carpet    │ │ • Concrete  ││
│  │ • Bright    │ │ • Dim light │ │ • Medium    │ │ • Dark      ││
│  │ • Lighting  │ │ • Lighting  │ │ • Lighting  │ │ • Lighting  ││
│  │ • Normal    │ │ • Different │ │ • Different │ │ • Different ││
│  │ • Objects   │ │ • Objects   │ │ • Objects   │ │ • Objects   ││
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘│
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              TRAINING PROCESS                           │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐ │   │
│  │  │   Model     │    │   Robust    │    │  Real-World │ │   │
│  │  │   Trained   │───►│   Model     │───►│  Performance│ │   │
│  │  │   on Single │    │   General   │    │  Improvement│ │   │
│  │  │   Scene     │    │   Purpose   │    │             │ │   │
│  │  └─────────────┘    └─────────────┘    └─────────────┘ │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  BENEFITS:                                                     │
│  • Improved robustness to environmental variations             │
│  • Better real-world transfer performance                      │
│  • Reduced overfitting to specific conditions                  │
│  • Enhanced model generalization capabilities                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram shows how domain randomization takes a base scene and creates multiple variations with different visual properties, leading to more robust and generalizable models.

## Figure SDG-004: Ground Truth Label Generation Process

```
┌─────────────────────────────────────────────────────────────────┐
│              GROUND TRUTH LABEL GENERATION                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  SIMULATION ENGINE KNOWLEDGE                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ • Object identities & properties                        │   │
│  │ • 3D positions & orientations                           │   │
│  │ • Material properties & textures                        │   │
│  │ • Lighting conditions & effects                         │   │
│  │ • Sensor positions & parameters                         │   │
│  │ • Physics interactions & collisions                     │   │
│  └─────────────────────────────────────────────────────────┘   │
│            │                                                   │
│            v                                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              SENSOR SIMULATION                          │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐      │   │
│  │  │   Camera    │ │    LiDAR    │ │    IMU      │      │   │
│  │  │             │ │             │ │             │      │   │
│  │  │ • RGB Data  │ │ • Point     │ │ • Acceler-  │      │   │
│  │  │ • Depth     │ │ • Cloud     │ │ • ometer    │      │   │
│  │  │ • Normals   │ │ • Data      │ │ • Gyro      │      │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘      │   │
│  └─────────────────────────────────────────────────────────┘   │
│            │                    │                    │         │
│            v                    v                    v         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │  SEMANTIC   │    │  INSTANCE   │    │   DEPTH     │        │
│  │  SEGMENTATION│    │SEGMENTATION │    │    MAPS     │        │
│  │             │    │             │    │             │        │
│  │ • Class IDs │    │ • Object    │    │ • Distance  │        │
│  │ • Pixel     │    │ • Instance  │    │ • To Camera │        │
│  │ • Labels    │    │ • IDs       │    │ • Values    │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│            │                    │                    │         │
│            v                    v                    v         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              DATASET ASSEMBLY                           │   │
│  │  • Synchronized multi-modal data                       │   │
│  │  • Precise temporal alignment                          │   │
│  │  • Quality validation & verification                   │   │
│  │  • Format standardization for ML training              │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates the process of automatic ground truth generation in simulation, showing how the simulation engine's complete knowledge is used to generate various types of precise labels.

## Figure SDG-005: Sim-to-Real Transfer Challenges

```
┌─────────────────────────────────────────────────────────────────┐
│                SIM-TO-REAL TRANSFER CHALLENGES                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  DOMAIN GAP ANALYSIS                                           │
│                                                                 │
│  SYNTHETIC DOMAIN                REAL-WORLD DOMAIN             │
│  ┌─────────────────┐            ┌─────────────────┐           │
│  │ • Perfect       │            │ • Sensor noise  │           │
│  │ • Lighting      │   GAP      │ • Imperfections │           │
│  │ • Textures      │   ────────►│ • Environmental │           │
│  │ • No noise      │            │ • Variations    │           │
│  │ • Consistent    │            │ • Uncontrolled  │           │
│  │ • Controlled    │            │ • Dynamic       │           │
│  │ • Parameters    │            │ • Conditions    │           │
│  └─────────────────┘            └─────────────────┘           │
│                                                                 │
│  CHALLENGE CATEGORIES:                                         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ VISUAL DIFFERENCES     SENSOR CHARACTERISTICS         │   │
│  │ • Lighting variations   • Noise patterns              │   │
│  │ • Texture properties    • Resolution differences      │   │
│  │ • Color reproduction    • Dynamic range limitations   │   │
│  │ • Reflection models     • Calibration variations      │   │
│  └─────────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ ENVIRONMENTAL FACTORS    DYNAMIC EFFECTS              │   │
│  │ • Weather conditions    • Motion blur                 │   │
│  │ • Atmospheric effects   • Temporal sampling           │   │
│  │ • Uncontrolled scenes   • Dynamic lighting            │   │
│  │ • Real physics          • Real motion artifacts       │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  SOLUTION STRATEGIES:                                          │
│  • Domain randomization to increase synthetic diversity        │
│  • Fine-tuning on small real-world datasets                    │
│  • Adversarial domain adaptation techniques                      │
│  • Progressive complexity training approaches                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates the various challenges in transferring models from synthetic to real-world domains, categorizing the differences and showing potential solution strategies.