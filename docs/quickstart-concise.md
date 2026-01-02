# Concise Quickstart Guide

Get started with the Physical AI & Humanoid Robotics book UI quickly.

## Prerequisites

- Node.js 18 or higher
- npm or yarn package manager

## Installation

1. Install dependencies:
   ```bash
   npm install
   ```

2. Start development server:
   ```bash
   npm start
   ```

## Key Features

- **Dark Mode**: Default dark theme with light mode toggle
- **Responsive Design**: Works on mobile, tablet, and desktop
- **Progress Tracking**: Local storage-based progress tracking
- **Interactive Elements**: Code blocks, tooltips, collapsible sections

## Essential Components

### Navigation
- `BookLayout`: Two-column layout with sidebar
- `Sidebar`: Module and chapter navigation
- `Breadcrumbs`: Navigation path display

### Content
- `ProgressIndicator`: Shows reading progress
- `CodeBlock`: Code with copy functionality
- `CalloutBox`: Notes, warnings, and concepts

### Interactive
- `GlossaryTooltip`: Term definitions
- `CollapsibleSection`: Expandable content sections
- `ToggleButton`: Feature toggles

## Customization

### Colors
Update `tailwind.config.js` to modify the color palette.

### Typography
Font settings are in `docusaurus.config.js` and `src/css/custom.css`.

## Development

- Edit components in `src/components/`
- Add content in `docs/` directory
- Customize styling in `src/css/custom.css`