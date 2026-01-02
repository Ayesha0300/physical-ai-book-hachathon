# Quickstart Guide

Get started with the Physical AI & Humanoid Robotics book UI in minutes.

## Prerequisites

- Node.js 18 or higher
- npm or yarn package manager

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd physical-ai-book
   ```

2. Install dependencies:
   ```bash
   npm install
   # or
   yarn install
   ```

## Development

1. Start the development server:
   ```bash
   npm start
   # or
   yarn start
   ```

2. Open your browser to `http://localhost:3000` to view the site.

## Building for Production

To create a production build:

```bash
npm run build
# or
yarn build
```

The build will be created in the `build` directory and can be served using any static hosting service.

## Customization

### Changing Colors

Update the color palette in `tailwind.config.js` to modify the theme colors.

### Adding Content

Content is organized in the `docs/` directory by modules and chapters. Create new markdown files to add content.

### Adding Components

Custom components are located in the `src/components/` directory. Create new components following the existing patterns and import them in the appropriate places.

## Key Features

- **Dark Mode**: The UI defaults to dark mode with a toggle for light mode
- **Responsive Design**: Works on mobile, tablet, and desktop devices
- **Interactive Elements**: Includes tooltips, collapsible sections, and toggle buttons
- **Progress Tracking**: Local storage-based progress tracking for users
- **Accessibility**: Keyboard navigation and screen reader support