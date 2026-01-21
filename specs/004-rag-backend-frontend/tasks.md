# Tasks: RAG Backend-Frontend Integration

**Input**: Design documents from `/specs/004-rag-backend-frontend/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`
- Adjust paths based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure in backend/
- [X] T002 [P] Install Python dependencies (fastapi, uvicorn, python-dotenv, openai, qdrant-client) in backend/
- [X] T003 [P] Create .env file template in project root
- [X] T004 [P] Initialize package.json for backend with poetry/pyproject.toml
- [X] T005 Create frontend configuration for chatbot component in src/components/Chatbot/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Create configuration management in backend/config.py
- [X] T007 [P] Create Pydantic models for requests in backend/models/request.py
- [X] T008 [P] Create Pydantic models for responses in backend/models/response.py
- [X] T009 Create main FastAPI app in backend/main.py
- [X] T010 Create API router in backend/api.py
- [X] T011 Create agent module in backend/agent.py
- [X] T012 Create service layer structure in backend/services/
- [X] T013 Setup CORS middleware for frontend-backend communication in backend/main.py
- [X] T014 [P] Create error handling utilities in backend/utils/errors.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Query Documentation with AI Assistance (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about book content through a chat interface and receive AI-generated responses grounded in book content

**Independent Test**: Can be fully tested by sending a query to the chat endpoint and verifying that the response is grounded in the book content, delivering accurate and contextual answers.

### Implementation for User Story 1

- [X] T015 [P] [US1] Create chat request model extending ChatQuery in backend/models/request.py
- [X] T016 [P] [US1] Create chat response model extending ChatResponse in backend/models/response.py
- [X] T017 [US1] Implement basic chat endpoint in backend/api.py
- [X] T018 [US1] Create retrieval service in backend/services/retrieval.py
- [X] T019 [US1] Implement basic agent functionality in backend/agent.py
- [X] T020 [US1] Connect endpoint to agent and retrieval in backend/api.py
- [X] T021 [US1] Add basic validation for query parameters
- [X] T022 [US1] Create simple chatbot UI component in src/components/Chatbot/Chatbot.jsx
- [X] T023 [US1] Add frontend-backend communication logic to chatbot component
- [X] T024 [US1] Test end-to-end flow: frontend query → backend → agent → response

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Real-time Chat Interaction (Priority: P2)

**Goal**: Enable multi-turn conversations with the AI agent so users can ask follow-up questions and explore related topics in the documentation

**Independent Test**: Can be tested by conducting a multi-turn conversation with the agent and verifying that context is maintained across exchanges.

### Implementation for User Story 2

- [X] T025 [P] [US2] Create conversation context model in backend/models/conversation.py
- [X] T026 [US2] Implement conversation service in backend/services/conversation_service.py
- [X] T027 [US2] Update agent to maintain conversation context in backend/agent.py
- [X] T028 [US2] Add conversation_id support to chat endpoint in backend/api.py
- [X] T029 [US2] Enhance chatbot UI for multi-turn conversations in src/components/Chatbot/Chatbot.jsx
- [X] T030 [US2] Add conversation history display to frontend
- [X] T031 [US2] Test multi-turn conversation flow with context preservation

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Selective Content Querying (Priority: P3)

**Goal**: Allow users to restrict AI agent's responses to specific sections of the book or selected text to get more targeted answers

**Independent Test**: Can be tested by selecting specific text or sections and verifying that the agent's responses are limited to that content range.

### Implementation for User Story 3

- [X] T032 [P] [US3] Update retrieval service to support content restriction in backend/services/retrieval.py
- [X] T033 [US3] Modify agent to respect content restrictions in backend/agent.py
- [X] T034 [US3] Add restrict_to_selection parameter handling in backend/api.py
- [X] T035 [US3] Enhance chatbot UI to support text selection in src/components/Chatbot/Chatbot.jsx
- [X] T036 [US3] Add "section-only" query option to frontend
- [X] T037 [US3] Test selective content querying functionality

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T038 [P] Add comprehensive logging throughout backend in backend/utils/logging.py
- [X] T039 [P] Add error handling and validation across all endpoints
- [X] T040 [P] Add request/response validation middleware
- [X] T041 [P] Improve frontend error handling and loading states
- [X] T042 [P] Add frontend documentation for chatbot component
- [X] T043 [P] Add tests for backend endpoints in backend/tests/
- [X] T044 [P] Add integration tests for complete flows
- [X] T045 [P] Update docusaurus.config.js to include chatbot component
- [X] T046 [P] Run quickstart validation to ensure complete functionality

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
Task: "Create chat request model extending ChatQuery in backend/models/request.py"
Task: "Create chat response model extending ChatResponse in backend/models/response.py"

# Launch all implementation tasks for User Story 1 together:
Task: "Implement basic chat endpoint in backend/api.py"
Task: "Create retrieval service in backend/services/retrieval.py"
Task: "Implement basic agent functionality in backend/agent.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence