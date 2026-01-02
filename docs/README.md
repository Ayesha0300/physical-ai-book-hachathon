# Physical AI & Humanoid Robotics Book - Documentation

Welcome to the documentation for the Physical AI & Humanoid Robotics book UI. This project provides an interactive, accessible, and responsive interface for learning about robotics, AI, and humanoid systems.

## Features

- **Responsive Design**: Works on mobile, tablet, and desktop devices
- **Dark Mode**: Default dark theme with light mode option
- **Interactive Elements**: Glossary tooltips, collapsible sections, toggle buttons
- **Progress Tracking**: Local storage-based progress tracking
- **Accessibility**: Keyboard navigation, ARIA labels, and screen reader support
- **Performance Optimized**: Fast loading and smooth interactions

## Technology Stack

- **Framework**: Docusaurus v3
- **Styling**: Tailwind CSS with custom dark theme
- **Fonts**: Inter (headings and body text)
- **Icons**: SVG icons from Heroicons

## UI Components

The UI includes various components designed for technical content:

- Navigation components (Navbar, Sidebar, Breadcrumbs)
- Content components (BookLayout, ProgressIndicator, SectionAnchor)
- Interactive components (ToggleButtons, GlossaryTooltip, CollapsibleSection)
- Assessment components (Checklists, Criteria cards)
- Module overview components

## Customization

The UI can be customized by:

- Modifying the Tailwind configuration in `tailwind.config.js`
- Adjusting color schemes in the CSS variables
- Adding new MDX components in `src/theme/MDXComponents.js`

## Development

To run the project locally:

1. Install dependencies: `npm install`
2. Start the development server: `npm start`
3. Open `http://localhost:3000` in your browser

## Contributing

We welcome contributions to improve the UI and user experience. Please follow the standard fork-and-pull request workflow.