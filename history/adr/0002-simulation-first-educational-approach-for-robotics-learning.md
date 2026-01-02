# ADR-0002: Simulation-First Educational Approach for Robotics Learning

> **Scope**: This ADR documents the educational methodology decision to prioritize simulation-based learning before physical hardware interaction in the Physical AI & Humanoid Robotics book.

- **Status:** Accepted
- **Date:** 2025-12-24
- **Feature:** 001-digital-twin-sim
- **Context:** For an educational book targeting students learning robotics and humanoid systems, we need to establish a pedagogical approach that balances learning effectiveness with accessibility. The target audience includes students with varying levels of hardware access and safety concerns.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt a simulation-first educational approach with:
- **Primary Focus**: Start with Gazebo/Unity simulation environments
- **Safety Priority**: Ensure risk-free learning environment for all students
- **Cost Effectiveness**: Eliminate hardware requirements for initial learning
- **Progression Path**: Gradual transition from simulation to hardware deployment
- **Target Audience**: Students with varying hardware access and experience levels

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Lower barrier to entry for students without expensive hardware
- Safer learning environment without physical robot risks
- Cost-effective for educational institutions and individual students
- Faster iteration and experimentation cycles
- Consistent learning environment across different student setups
- Ability to simulate dangerous or complex scenarios safely
- Immediate feedback without hardware setup time

### Negative

- Potential disconnect between simulation and real-world physics
- Students may overestimate performance in simulation
- Hardware-specific challenges not apparent in simulation
- Possible reduced motivation for students wanting immediate physical results
- Simulation accuracy limitations may mislead students
- Additional complexity in explaining sim-to-real transfer

## Alternatives Considered

Alternative A: Hardware-First Approach
- Start with physical robots and basic hardware
- Introduce simulation later for advanced topics
- Why rejected: Higher cost, safety concerns, limited accessibility

Alternative B: Parallel Simulation and Hardware Approach
- Develop both simultaneously with equal emphasis
- Students work with both from the beginning
- Why rejected: Potentially overwhelming for beginners, higher resource requirements

Alternative C: Simulation-Only Approach
- Focus entirely on simulation without hardware introduction
- Why rejected: Students would lack real-world robotics experience

## References

- Feature Spec: specs/001-digital-twin-sim/spec.md
- Implementation Plan: specs/001-digital-twin-sim/plan.md
- Related ADRs: ADR-0001 (Digital Twin Simulation Architecture with Gazebo and Unity)
- Evaluator Evidence: specs/001-digital-twin-sim/research.md <!-- link to eval notes/PHR showing graders and outcomes -->
