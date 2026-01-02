# ADR-0004: Content Depth vs Breadth Strategy for Technical Book

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-24
- **Feature:** 002-isaac-ai-brain
- **Context:** The Physical AI & Humanoid Robotics book must balance comprehensive coverage of the robotics ecosystem with deep technical understanding of specific technologies. This decision impacts how each chapter will be structured and how much detail to include on each topic.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Focus on practical depth with sufficient breadth for context. Each chapter will provide detailed, actionable knowledge for specific technologies while maintaining awareness of the broader robotics ecosystem. This approach prioritizes enabling readers to implement and understand core concepts deeply while providing enough breadth to understand how different components integrate.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Readers gain practical, implementable knowledge in each chapter
- Deep understanding enables effective application of concepts
- Clear focus prevents content from becoming superficial
- Enables readers to build on foundational knowledge for advanced applications

### Negative

- May require readers to consult additional resources for comprehensive ecosystem coverage
- Some readers may desire broader exposure to more technologies
- Risk of information density overwhelming some learners
- May not suit readers seeking high-level overviews

## Alternatives Considered

- **Deep Dive Only**: Focus exclusively on detailed technical implementation without broader context - Rejected because it would limit understanding of how components integrate in real systems
- **Broad Overview Only**: Cover many technologies superficially to provide comprehensive ecosystem awareness - Rejected because it would fail to provide actionable, implementable knowledge
- **Equal Depth and Breadth**: Balance both equally in all chapters - Rejected because it would result in unwieldy content that fails to provide either deep knowledge or comprehensive coverage

## References

- Feature Spec: specs/002-isaac-ai-brain/spec.md
- Implementation Plan: specs/002-isaac-ai-brain/plan.md
- Related ADRs: ADR-0001, ADR-0002, ADR-0003
- Evaluator Evidence: specs/002-isaac-ai-brain/plan.md#decisions-needing-documentation
