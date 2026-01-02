# ADR-0001: Digital Twin Simulation Architecture with Gazebo and Unity

> **Scope**: This ADR documents the integrated decision to use Gazebo for physics simulation and Unity for visualization in the digital twin simulation architecture for the Physical AI & Humanoid Robotics book.

- **Status:** Accepted
- **Date:** 2025-12-24
- **Feature:** 001-digital-twin-sim
- **Context:** For the digital twin simulation module of the Physical AI & Humanoid Robotics educational book, we need to establish a simulation architecture that balances physics accuracy with visualization quality. The book targets students learning robotics concepts, requiring a safe, cost-effective environment for experimentation before hardware deployment.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use a dual-simulation approach with:
- **Physics Simulation**: Gazebo for accurate physics modeling, sensor simulation, and environment interaction
- **Visualization**: Unity for high-quality 3D rendering, user interaction, and immersive visualization
- **Integration**: ROS2 bridges to connect both simulation environments seamlessly
- **Target Audience**: Educational use for students learning robotics and humanoid systems

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Leverages Gazebo's superior physics accuracy and established robotics simulation capabilities
- Provides Unity's high-quality visualization and user experience for better student engagement
- Enables comprehensive simulation that covers both physics and visual aspects of robotics
- Supports safe, cost-effective learning environment without requiring expensive hardware
- Allows for realistic sensor simulation and environment modeling
- Facilitates sim-to-real transfer learning for students

### Negative

- Increased complexity due to dual-simulation environment
- Potential synchronization challenges between Gazebo and Unity
- Higher resource requirements for running both simulation platforms
- More complex integration and maintenance requirements
- Students need to understand both simulation systems
- Potential licensing costs for Unity Professional if needed

## Alternatives Considered

Alternative A: Gazebo Only Approach
- Physics: Gazebo for everything
- Visualization: Gazebo's built-in visualization
- Why rejected: Less visually appealing, potentially less engaging for students

Alternative B: Unity Only Approach
- Physics: Unity's physics engine
- Visualization: Unity's rendering system
- Why rejected: Less accurate physics modeling for robotics applications

Alternative C: Single Simulation Platform (Isaac Sim)
- Physics: Isaac Sim's physics engine
- Visualization: Isaac Sim's rendering
- Why rejected: Higher barrier to entry, less flexibility, more vendor-specific approach

## References

- Feature Spec: specs/001-digital-twin-sim/spec.md
- Implementation Plan: specs/001-digital-twin-sim/plan.md
- Related ADRs: None
- Evaluator Evidence: specs/001-digital-twin-sim/research.md <!-- link to eval notes/PHR showing graders and outcomes -->
