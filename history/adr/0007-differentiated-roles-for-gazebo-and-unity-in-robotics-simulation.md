# ADR-0007: Differentiated Roles for Gazebo and Unity in Robotics Simulation

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-24
- **Feature:** 002-isaac-ai-brain
- **Context:** The Physical AI & Humanoid Robotics book must define distinct roles for Gazebo and Unity simulation platforms. This decision impacts how students will learn to choose appropriate simulation tools for different robotics applications and use cases.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Define differentiated roles with clear use case definitions: Gazebo for physics-based simulation and realistic robotics environments, Unity for visualization, photorealistic rendering, and game-engine-based simulation. This approach enables students to understand when and why to use each platform based on their specific simulation requirements.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Students learn to select appropriate simulation tools for specific needs
- Clear understanding of each platform's strengths and optimal use cases
- Enables comprehensive coverage of robotics simulation approaches
- Provides practical guidance for real-world simulation decisions
- Maximizes learning value by leveraging each platform's unique capabilities
- Prepares students for industry scenarios where both platforms are commonly used

### Negative

- Requires students to learn two different simulation platforms
- May create confusion about which platform to use for specific scenarios
- Increases content complexity and learning curve
- Potential for students to develop preferences that don't align with optimal choices
- Requires more time to cover both platforms comprehensively

## Alternatives Considered

- **Gazebo-Only Approach**: Focus exclusively on Gazebo for all simulation needs - Rejected because it would limit exposure to photorealistic simulation capabilities essential for modern robotics
- **Unity-Only Approach**: Focus exclusively on Unity for all simulation needs - Rejected because it would miss Gazebo's established robotics ecosystem and physics capabilities
- **Equal Treatment Approach**: Treat both platforms as equivalent with no differentiated roles - Rejected because it would fail to guide students toward optimal platform selection

## References

- Feature Spec: specs/002-isaac-ai-brain/spec.md
- Implementation Plan: specs/002-isaac-ai-brain/plan.md
- Related ADRs: ADR-0001, ADR-0002, ADR-0004, ADR-0005, ADR-0006
- Evaluator Evidence: specs/002-isaac-ai-brain/plan.md#decisions-needing-documentation
