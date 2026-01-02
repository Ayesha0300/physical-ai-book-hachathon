# Component Reference Guide

This document provides a comprehensive reference for all UI components in the Physical AI & Humanoid Robotics book interface.

## Navigation Components

### BookLayout
**Location**: `src/components/BookLayout/BookLayout.js`
**Purpose**: Main layout component with two-column design
**Props**:
- `children`: Content to be displayed
- `title`: Page title (optional)

### Sidebar
**Location**: `src/components/Navigation/Sidebar.js`
**Purpose**: Navigation sidebar with collapsible sections
**Props**: None

### Breadcrumbs
**Location**: `src/components/Navigation/Breadcrumbs.js`
**Purpose**: Shows navigation path for current page
**Props**:
- `items`: Array of navigation items `[{label: string, href: string}]`

## Content Components

### ProgressIndicator
**Location**: `src/components/ProgressIndicator.js`
**Purpose**: Shows progress through a chapter or module
**Props**:
- `current`: Current progress value
- `total`: Total possible value
- `chapterTitle`: Title to display (optional)

### SectionAnchor
**Location**: `src/components/SectionAnchor.js`
**Purpose**: Creates anchor links for sections
**Props**:
- `id`: Anchor ID
- `title`: Section title
- `children`: Content to wrap

### CodeBlock
**Location**: `src/components/CodeBlock.js`
**Purpose**: Displays code with syntax highlighting and copy functionality
**Props**:
- `children`: Code content
- `language`: Programming language (default: "")
- `showLineNumbers`: Show line numbers (default: false)

### CalloutBox
**Location**: `src/components/CalloutBox.js`
**Purpose**: Styled boxes for notes, warnings, and concepts
**Props**:
- `type`: Type of callout ("note", "warning", "concept")
- `title`: Title for the callout (optional)
- `children`: Content to display

## Interactive Components

### ToggleButton
**Location**: `src/components/InteractiveElements/ToggleButton.js`
**Purpose**: Toggle switch for enabling/disabling features
**Props**:
- `label`: Label text
- `checked`: Initial state (default: false)
- `onChange`: Callback function
- `disabled`: Disable state (default: false)

### GlossaryTooltip
**Location**: `src/components/GlossaryTooltip.js`
**Purpose**: Tooltip with definitions for technical terms
**Props**:
- `term`: Term being defined
- `definition`: Definition text
- `children`: Trigger element

### CollapsibleSection
**Location**: `src/components/CollapsibleSection.js`
**Purpose**: Section that can be expanded/collapsed
**Props**:
- `title`: Section title
- `children`: Content to show/hide
- `defaultOpen`: Initial state (default: false)

## Module Components

### ModuleOverview
**Location**: `src/components/ModuleOverview.js`
**Purpose**: Displays module information and learning outcomes
**Props**:
- `title`: Module title
- `description`: Module description
- `learningOutcomes`: Array of learning outcomes
- `timeline`: Timeline information
- `requirements`: Array of requirements

### HardwareRequirementsCard
**Location**: `src/components/HardwareRequirementsCard.js`
**Purpose**: Displays hardware/software requirements in card format
**Props**:
- `title`: Card title
- `description`: Card description
- `requirements`: Array of requirements
- `type`: "hardware" or "software" (default: "hardware")

### AssessmentChecklist
**Location**: `src/components/AssessmentChecklist.js`
**Purpose**: Interactive checklist for assessments
**Props**:
- `title`: Checklist title
- `items`: Array of checklist items
- `description`: Checklist description

### SubmissionCriteriaCard
**Location**: `src/components/SubmissionCriteriaCard.js`
**Purpose**: Displays submission criteria and evaluation methods
**Props**:
- `title`: Card title
- `criteria`: Array of criteria
- `evaluationMethod`: Evaluation method description
- `deadline`: Deadline information

### WeeklyTimeline
**Location**: `src/components/WeeklyTimeline.js`
**Purpose**: Displays weekly breakdown of content
**Props**:
- `weeks`: Array of week objects `[{title: string, description: string, topics: array}]`

## Utility Components

### VisualSystemDiagram
**Location**: `src/components/VisualSystemDiagram.js`
**Purpose**: Placeholder for system diagrams
**Props**:
- `title`: Diagram title
- `description`: Diagram description
- `children`: Additional content

### CapstoneLayout
**Location**: `src/components/CapstoneLayout.js`
**Purpose**: Layout for capstone projects
**Props**:
- `title`: Layout title
- `children`: Content to display
- `steps`: Array of steps for the project

### LoadingSpinner
**Location**: `src/components/LoadingSpinner.js`
**Purpose**: Loading indicator with different sizes
**Props**:
- `size`: Size of spinner ("sm", "md", "lg", "xl") - default: "md"
- `label`: Label text (default: "Loading...")

## Progress Tracking

### ProgressTracker
**Location**: `src/components/ProgressTracker.js`
**Purpose**: Utility for tracking and persisting user progress
**Functions**:
- `getProgress(contentId)`: Get progress for content
- `setProgress(contentId, progress)`: Set progress for content
- `getModuleProgress(moduleId)`: Get all progress for a module
- `calculateModuleProgress(moduleId)`: Calculate overall module progress

## Hooks

### useProgressTracker
**Location**: `src/components/ProgressTracker.js`
**Purpose**: React hook for managing progress state
**Usage**: `const { progress, updateProgress, markComplete } = useProgressTracker(contentId, initialProgress)`

## Global Components

### GlobalKeyboardNavigation
**Location**: `src/components/GlobalKeyboardNavigation.js`
**Purpose**: Provides global keyboard navigation functionality
**Usage**: Include once in main layout