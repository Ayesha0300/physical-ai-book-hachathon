# Diagrams for Nav2 Navigation Concepts

## Figure NAV2-001: Navigation Problem for Humanoid Robots

```
┌─────────────────────────────────────────────────────────────────┐
│              NAVIGATION PROBLEM FOR HUMANOID ROBOTS           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  HUMANOID ROBOT NAVIGATION SCENARIO                           │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    ENVIRONMENT                          │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │   │
│  │  │   Human     │  │  Furniture  │  │   Door     │   │   │
│  │  │             │  │             │  │            │   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘   │   │
│  │         │                  │                  │         │   │
│  │         └──────────────────┼──────────────────┘         │   │
│  │                            │                            │   │
│  │         ┌─────────────────────────────────────────┐     │   │
│  │         │            NAVIGATION PATH            │     │   │
│  │         │  ┌──┐    ┌──┐    ┌──┐    ┌──┐      │     │   │
│  │         │  │S │────→│   │───→│   │───→│G │      │     │   │
│  │         │  └──┘    └──┘    └──┘    └──┘      │     │   │
│  │         │  Start    Waypoint Waypoint  Goal    │     │   │
│  │         │  Position  1       2       Position │     │   │
│  │         └─────────────────────────────────────────┘     │   │
│  │                                                         │   │
│  │  HUMANOID ROBOT ──────────────────────────────→ ACTIONS │   │
│  │  ┌─────────────────┐                                   │   │
│  │  │  • Bipedal      │  ┌─────────────────────────────┐ │   │
│  │  │    locomotion   │  │  • Balance control         │ │   │
│  │  │  • Human-scale  │  │  • Footstep planning       │ │   │
│  │  │  • Safety       │  │  • Social navigation       │ │   │
│  │  │    requirements │  │  • Energy efficiency       │ │   │
│  │  └─────────────────┘  └─────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  CHALLENGES:                                                  │
│  • Dynamic balance maintenance during navigation               │
│  • Human-scale environment compatibility                       │
│  • Social navigation in human environments                     │
│  • Safety near humans                                          │
│  • Energy-efficient bipedal locomotion                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates the navigation problem for humanoid robots, showing how they must navigate through environments with humans and obstacles while maintaining balance and following social norms.

## Figure NAV2-002: Nav2 Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    NAV2 ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              NAVIGATION SERVER                          │   │
│  │  ┌─────────────────────────────────────────────────┐   │   │
│  │  │  • State Management                           │   │   │
│  │  │  • Goal Processing                            │   │   │
│  │  │  • Component Coordination                     │   │   │
│  │  │  • Event Publishing                           │   │   │
│  │  └─────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────┘   │
│            │                                                   │
│            v                                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              BEHAVIOR TREES                             │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │   │
│  │  │  Navigation │  │  Recovery   │  │  Utility   │   │   │
│  │  │  Tree       │  │  Behaviors  │  │  Nodes     │   │   │
│  │  │             │  │             │  │             │   │   │
│  │  │ • Planning  │  │ • Spin      │  │ • Sequences │   │   │
│  │  │ • Execution │  │ • Backup    │  │ • Fallbacks │   │   │
│  │  │ • Recovery  │  │ • Wait      │  │ • Decorators│   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘   │   │
│  └─────────────────────────────────────────────────────────┘   │
│            │                    │                    │         │
│            v                    v                    v         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │  PLANNERS   │    │  CONTROLLERS│    │  RECOVERY  │        │
│  │             │    │             │    │  BEHAVIORS │        │
│  │ • Global    │    │ • Local     │    │             │        │
│  │ • Local     │    │ • DWB       │    │ • Spin      │        │
│  │ • A*        │    │ • TEB       │    │ • Backup    │        │
│  │ • NavFn     │    │ • MPC       │    │ • Wait      │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│            │                    │                    │         │
│            v                    v                    v         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              MOTION PRIMITIVES                          │   │
│  │  ┌─────────────────────────────────────────────────┐   │   │
│  │  │  • Action Interfaces                          │   │   │
│  │  │  • Robot Abstraction                          │   │   │
│  │  │  • Safety Integration                         │   │   │
│  │  │  • Feedback Processing                        │   │   │
│  │  └─────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  INTEGRATION POINTS:                                           │
│  • ROS 2 communication patterns                                │
│  • Standard message types                                      │
│  • Parameter configuration                                     │
│  • Lifecycle management                                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram shows the overall Nav2 architecture, highlighting the central Navigation Server, Behavior Trees for task management, and the different components for planning, control, and recovery behaviors.

## Figure NAV2-003: Global Planning vs Local Control

```
┌─────────────────────────────────────────────────────────────────┐
│           GLOBAL PLANNING vs LOCAL CONTROL                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  GLOBAL PLANNING                    LOCAL CONTROL              │
│  ┌─────────────────────────┐    ┌─────────────────────────┐   │
│  │ • High-level path       │    │ • Short-term trajectory │   │
│  │ • Static map based      │    │ • Dynamic obstacle      │   │
│  │ • Optimality focus      │    │   avoidance             │   │
│  │ • Long-term route       │    │ • Path following        │   │
│  │ • Constraint handling   │    │ • Real-time execution   │   │
│  │ • Path smoothing        │    │ • Safety enforcement    │   │
│  │ • Infrequent updates    │    │ • Frequent updates      │   │
│  └─────────────────────────┘    └─────────────────────────┘   │
│                                                                 │
│  PLANNING PROCESS:                                             │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  START ──→ GLOBAL PLAN ──→ LOCAL EXECUTION ──→ GOAL   │   │
│  │   │           │                    │              │     │   │
│  │   │           │                    │              │     │   │
│  │   │    ┌─────────────┐    ┌─────────────┐        │     │   │
│  │   │    │  PATH       │    │  TRAJECTORY │        │     │   │
│  │   │    │  OPTIMIZER  │    │  TRACKER    │        │     │   │
│  │   │    └─────────────┘    └─────────────┘        │     │   │
│  │   │           │                    │              │     │   │
│  │   └───────────┼────────────────────┼──────────────┘     │   │
│  │               │                    │                    │   │
│  │         ┌─────────────┐    ┌─────────────┐            │   │
│  │         │  MAP        │    │  SENSORS    │            │   │
│  │         │  (Static)   │    │  (Dynamic)  │            │   │
│  │         └─────────────┘    └─────────────┘            │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  COORDINATION POINTS:                                          │
│  • Path resolution matching                                    │
│  • Update frequency coordination                               │
│  • Feedback integration                                        │
│  • Replanning triggers                                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram contrasts global planning (long-term, map-based path planning) with local control (short-term, obstacle-avoidance trajectory execution) and shows how they work together in the navigation pipeline.

## Figure NAV2-004: Humanoid-Specific Navigation Adaptations

```
┌─────────────────────────────────────────────────────────────────┐
│         HUMANOID-SPECIFIC NAVIGATION ADAPTATIONS              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  STANDARD NAV2 ──────────────────────────────────────→ HUMANOID│
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────┐ │
│  │  Wheeled    │ │  Static     │ │  Simple     │ │  Basic  │ │
│  │  Navigation │ │  Planning    │ │  Control    │ │  Safety │ │
│  │             │ │             │ │             │ │         │ │
│  │ • Smooth    │ │ • 2D paths  │ │ • Velocity  │ │ • Stop  │ │
│  │ • Turning   │ │ • Simple    │ │ • PID       │ │ • Avoid │ │
│  │ • Continuous│ │ • Static    │ │ • Direct    │ │ • Basic │ │
│  │ • Stable    │ │ • Fast      │ │ • Fast      │ │ • Fast  │ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────┘ │
│         │              │              │              │         │
│         v              v              v              v         │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────┐ │
│  │  Bipedal    │ │  Humanoid-  │ │  Balance-  │ │  Human- │ │
│  │  Navigation │ │  Aware      │ │  Aware     │ │  Aware  │ │
│  │             │ │  Planning   │ │  Control   │ │  Safety │ │
│  │ • Step-by-  │ │ • Kinematic │ │ • ZMP       │ │ • Social│ │
│  │ •   step    │ │ •   constraints││ • Capture  │ │ • Proxemics││
│  │ • Balance   │ │ • Stair     │ │ •   Point   │ │ • Safe  │ │
│  │ • Dynamic   │ │ • Social    │ │ • Whole-    │ │ • Human │ │
│  │ • Complex   │ │ • Accessible│ │ •   body    │ │ • Predict││
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────┘ │
│                                                                 │
│  KEY ADAPTATIONS:                                              │
│  • Footstep planning integration                               │
│  • Balance-aware path execution                                │
│  • Human-aware obstacle avoidance                              │
│  • Social navigation compliance                                │
│  • Energy-efficient gait planning                              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram shows the key adaptations needed to transform standard Nav2 for humanoid robots, highlighting the differences in navigation, planning, control, and safety approaches.

## Figure NAV2-005: Safety and Human-Aware Navigation

```
┌─────────────────────────────────────────────────────────────────┐
│         SAFETY AND HUMAN-AWARE NAVIGATION                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  HUMAN ENVIRONMENT NAVIGATION                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    NAVIGATION SPACE                     │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │   │
│  │  │   Human A   │  │   Human B   │  │   Human C   │   │   │
│  │  │  (Personal  │  │  (Social    │  │  (Public    │   │   │
│  │  │   Space)    │  │   Space)    │  │   Space)    │   │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘   │   │
│  │         │                  │                  │         │   │
│  │         └──────────────────┼──────────────────┘         │   │
│  │                            │                            │   │
│  │         ┌─────────────────────────────────────────┐     │   │
│  │         │            SAFE NAVIGATION PATH       │     │   │
│  │         │  ┌──┐    ┌──┐    ┌──┐    ┌──┐      │     │   │
│  │         │  │  │────→│  │───→│  │───→│  │      │     │   │
│  │         │  │R │    │R │    │R │    │R │      │     │   │
│  │         │  │o │    │o │    │o │    │o │      │     │   │
│  │         │  │b │    │b │    │b │    │b │      │     │   │
│  │         │  │o │    │o │    │o │    │o │      │     │   │
│  │         │  │t │    │t │    │t │    │t │      │     │   │
│  │         │  └──┘    └──┘    └──┘    └──┘      │     │   │
│  │         │  Respects  Avoids  Respects  Approaches│     │   │
│  │         │  Personal  Humans  Social  Interaction│     │   │
│  │         └─────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  SAFETY SYSTEM LAYERS:                                         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  EMERGENCY LAYER: Immediate stop if collision imminent│   │
│  │  ──────────────────────────────────────────────────────│   │
│  │  REACTIVE LAYER: Evasive maneuvers and path adjustment│   │
│  │  ──────────────────────────────────────────────────────│   │
│  │  PROACTIVE LAYER: Predictive path planning with safety│   │
│  │  ──────────────────────────────────────────────────────│   │
│  │  SOCIAL LAYER: Follow human navigation norms and      │   │
│  │  ──────────────────────────────────────────────────────│   │
│  │  BASE LAYER: Basic obstacle avoidance and navigation  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  HUMAN AWARENESS FEATURES:                                     │
│  • Personal space respect (45cm-1.2m)                        │
│  • Social navigation norms (passing, approaching)              │
│  • Human motion prediction                                     │
│  • Intent recognition                                          │
│  • Emergency response (human distress)                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

Description: This diagram illustrates safety and human-aware navigation concepts, showing how humanoid robots must navigate in human environments while respecting personal space and social norms, with multiple safety system layers.