# ADR-0005: Simulation-First Educational Approach for Robotics Learning

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-24
- **Feature:** 002-isaac-ai-brain
- **Context:** The Physical AI & Humanoid Robotics book must decide whether to prioritize simulation tools and environments or real hardware approaches in the learning progression. This decision impacts accessibility, cost, and the learning sequence for students.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt a simulation-first approach with clear pathways to hardware integration. Students will first learn concepts and implementation using simulation environments (Isaac Sim, Gazebo, Unity) before transitioning to hardware. This approach prioritizes accessibility and rapid iteration while maintaining clear connections to real-world applications.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Enables learning without expensive hardware requirements
- Allows rapid iteration and experimentation
- Provides safe environment for testing complex behaviors
- Facilitates reproducible examples and exercises
- Reduces barrier to entry for diverse student population
- Enables focus on algorithmic and conceptual understanding before hardware constraints

### Negative

- May create disconnect between simulation and real-world behavior
- Risk of overfitting to simulation environments
- Students may underestimate real-world complexity and constraints
- Potential for simulation-to-reality gap in learned behaviors
- Requires careful design to ensure transferability of knowledge

## Alternatives Considered

- **Hardware-First Approach**: Begin with real hardware to ground learning in physical reality - Rejected because it creates high barriers to entry and limits experimentation opportunities
- **Parallel Simulation and Hardware**: Teach both simultaneously - Rejected because it increases cognitive load and complexity without clear benefit
- **Hardware-Only Approach**: Focus exclusively on real hardware platforms - Rejected because of accessibility, safety, and cost constraints

## References

- Feature Spec: specs/002-isaac-ai-brain/spec.md
- Implementation Plan: specs/002-isaac-ai-brain/plan.md
- Related ADRs: ADR-0001, ADR-0002, ADR-0004
- Evaluator Evidence: specs/002-isaac-ai-brain/plan.md#decisions-needing-documentation
