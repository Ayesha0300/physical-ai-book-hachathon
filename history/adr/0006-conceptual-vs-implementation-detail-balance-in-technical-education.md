# ADR-0006: Conceptual vs Implementation Detail Balance in Technical Education

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-24
- **Feature:** 002-isaac-ai-brain
- **Context:** The Physical AI & Humanoid Robotics book must balance theoretical understanding with practical implementation skills. This decision impacts how each chapter will present concepts versus hands-on examples and code.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt a balanced approach with clear progression from concepts to implementation. Each chapter will begin with conceptual understanding and theoretical foundations, then progress to practical implementation examples. This approach ensures students understand both "why" and "how" of each technology or concept.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Students develop both theoretical understanding and practical skills
- Clear progression from abstract concepts to concrete implementations
- Enables deeper comprehension through hands-on application
- Provides multiple learning pathways for different learning styles
- Reinforces conceptual knowledge through practical application
- Builds confidence through tangible, working examples

### Negative

- May require more time to cover each topic comprehensively
- Risk of overwhelming students with too much information at once
- Requires careful balance to avoid either extreme (pure theory or pure practice)
- May not suit learners who prefer exclusively theoretical or practical approaches
- Increases content complexity and length

## Alternatives Considered

- **Theory-Heavy Approach**: Focus primarily on conceptual understanding with minimal implementation - Rejected because it would fail to provide practical skills needed for implementation
- **Code-Heavy Approach**: Focus primarily on implementation examples with minimal conceptual explanation - Rejected because it would lack foundational understanding needed for effective application
- **Parallel Approach**: Present concepts and implementation simultaneously without clear progression - Rejected because it would increase cognitive load and reduce comprehension

## References

- Feature Spec: specs/002-isaac-ai-brain/spec.md
- Implementation Plan: specs/002-isaac-ai-brain/plan.md
- Related ADRs: ADR-0001, ADR-0002, ADR-0004, ADR-0005
- Evaluator Evidence: specs/002-isaac-ai-brain/plan.md#decisions-needing-documentation
