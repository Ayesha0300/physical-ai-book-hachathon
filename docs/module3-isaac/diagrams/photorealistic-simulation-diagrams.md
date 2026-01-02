# Diagrams for Photorealistic Simulation Concepts

## Figure PS-001: Isaac Sim Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    ISAAC SIM ARCHITECTURE                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │  Omniverse  │    │  PhysX      │    │  RTX Ray    │        │
│  │  Platform   │◄──►│  Physics    │◄──►│  Tracing    │        │
│  │             │    │  Engine     │    │  Engine     │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│         │                   │                   │              │
│         │                   │                   │              │
│         v                   v                   v              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │  Robotics   │    │  Physics    │    │  Visual     │        │
│  │  Framework  │    │  Simulation │    │  Rendering  │        │
│  │             │    │             │    │             │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram shows the core components of Isaac Sim, highlighting how Omniverse provides the platform foundation, PhysX handles physics simulation, and RTX enables photorealistic rendering.

## Figure PS-002: Reality Gap Comparison

```
┌─────────────────────────────────────────────────────────────────┐
│                    REALITY GAP ILLUSTRATION                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  TRADITIONAL SIMULATION        │        ISAAC SIM              │
│  ┌─────────────────────────┐   │   ┌─────────────────────────┐ │
│  │  Simplified Graphics    │   │   │  Photorealistic       │ │
│  │  + Basic Physics       │   │   │  Rendering + Accurate  │ │
│  │                        │   │   │  Physics              │ │
│  │  [Simple Cube Robot]   │   │   │  [Detailed Robot]    │ │
│  │  [Flat Colors]        │   │   │  [Realistic Textures]│ │
│  │  [Basic Lighting]     │   │   │  [RTX Lighting]      │ │
│  │                        │   │   │                       │ │
│  │  Performance: High    │   │   │  Performance: Medium  │ │
│  │  Reality Gap: Large   │   │   │  Reality Gap: Small   │ │
│  └─────────────────────────┘   │   └─────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates the difference between traditional simulators with simplified graphics and Isaac Sim's photorealistic approach, showing how the latter significantly reduces the reality gap.

## Figure PS-003: Domain Randomization Process

```
┌─────────────────────────────────────────────────────────────────┐
│                 DOMAIN RANDOMIZATION PROCESS                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  FIXED ENVIRONMENT → RANDOMIZED ENVIRONMENTS                   │
│                                                                 │
│  Base Scene: Kitchen                                            │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐│
│  │  Default    │ │  Variation  │ │  Variation  │ │  Variation  ││
│  │  Kitchen    │ │     1       │ │     2       │ │     3       ││
│  │             │ │             │ │             │ │             ││
│  │  Blue Walls │ │  Red Walls  │ │ Green Walls │ │  Gray Walls ││
│  │  Wooden    │ │  Marble     │ │  Tile      │ │  Concrete   ││
│  │  Floor     │ │  Floor      │ │  Floor     │ │  Floor      ││
│  │  Bright    │ │  Dim       │ │  Medium    │ │  Dark      ││
│  │  Lighting  │ │  Lighting   │ │  Lighting  │ │  Lighting   ││
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘│
│                                                                 │
│  Training Result: AI Model Robust to Environmental Variations   │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram shows how domain randomization takes a base environment and creates multiple variations with different visual properties to improve model robustness.

## Figure PS-004: Physics vs Visual Fidelity Trade-offs

```
┌─────────────────────────────────────────────────────────────────┐
│              FIDELITY TRADE-OFFS ANALYSIS                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  PERFORMANCE vs FIDELITY                                        │
│                                                                 │
│  High │                    • Physics Focus                     │
│  Perf│                   • • Visual Focus                      │
│  o   │                  • • • Balanced                        │
│  r   │                 • • • •                                │
│  m   │                • • • • •                               │
│  a   │               • • • • • •                              │
│  n   │              • • • • • • •                             │
│  c   │             • • • • • • • •                            │
│  e   │            • • • • • • • • •                           │
│      │           • • • • • • • • • •                          │
│      │          • • • • • • • • • • •                         │
│      │         • • • • • • • • • • • •                        │
│      │        • • • • • • • • • • • • •                       │
│      │       • • • • • • • • • • • • • •                      │
│      │      • • • • • • • • • • • • • • •                     │
│      │     • • • • • • • • • • • • • • • •                    │
│      │    • • • • • • • • • • • • • • • • •                   │
│      │   • • • • • • • • • • • • • • • • • •                  │
│      │  • • • • • • • • • • • • • • • • • • •                 │
│      │ • • • • • • • • • • • • • • • • • • • •                │
│  Low │──────────────────────────────────────────────────────────│
│      │ Low                  FIDELITY LEVEL                   High│
│         Physics Focused                    Visual Focused        │
│                                                                 │
│  Optimal Balance: Depends on Application Requirements           │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates the trade-off between physics fidelity and visual fidelity, showing how computational resources can be allocated differently based on application needs.

## Figure PS-005: Scene Construction for Humanoid Robots

```
┌─────────────────────────────────────────────────────────────────┐
│              HUMANOID ROBOT SCENE LAYOUT                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                HUMANOID TRAINING SPACE                      ││
│  │  ┌─────────┐         ┌─────────────────┐                   ││
│  │  │  Human  │         │   Stairs/Steps  │                   ││
│  │  │  Sized  │         │                 │                   ││
│  │  │Furniture│         │     [Test]      │                   ││
│  │  └─────────┘         └─────────────────┘                   ││
│  │      │                        │                             ││
│  │      │                        │                             ││
│  │  ┌─────────┐            ┌─────────────┐                     ││
│  │  │  Door-  │            │ Navigation  │                     ││
│  │  │  ways   │◄──────────►│   Obstacles │                     ││
│  │  │(Human-  │            │             │                     ││
│  │  │ sized)  │            │  [Chairs,   │                     ││
│  │  └─────────┘            │  Tables]    │                     ││
│  │                         └─────────────┘                     ││
│  │                              │                              ││
│  │  ┌─────────────────────────────────────────────────────────┐││
│  │  │                    Interaction Zone                    │││
│  │  │  ┌──────────┐    ┌──────────┐    ┌──────────┐         │││
│  │  │  │ Manip-   │    │ Balance  │    │ Human    │         │││
│  │  │  │ ulable   │    │  Tests   │    │  Space   │         │││
│  │  │  │  Items   │    │          │    │          │         │││
│  │  │  └──────────┘    └──────────┘    └──────────┘         │││
│  │  └─────────────────────────────────────────────────────────┘││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                 │
│  Key Features: Human-scale dimensions, navigation challenges,   │
│  interaction objects, balance testing areas                     │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates the key components of a scene designed specifically for humanoid robot training, including human-scale furniture, navigation challenges, and interaction zones.