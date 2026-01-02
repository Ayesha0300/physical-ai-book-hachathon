# Quickstart Guide: Physical AI Platform – Landing Page & UI/UX Stabilization

## Development Setup

1. **Prerequisites**
   - Node.js 18+ installed
   - npm or yarn package manager

2. **Installation**
   ```bash
   # Navigate to project directory
   cd physical-ai-book

   # Install dependencies
   npm install
   ```

3. **Development Server**
   ```bash
   # Start development server
   npm run start

   # The site will be available at http://localhost:3000
   ```

## Key Components to Modify

1. **Landing Page** (`src/pages/index.js`)
   - Update hero section with new title and subtitle
   - Add curriculum overview section
   - Implement learning path preview

2. **Navigation** (`src/theme/Navbar.js`)
   - Fix active state highlighting
   - Update navigation links to resolve broken routes

3. **Module Cards** (`src/components/Modules/ModuleCard.js`)
   - Create responsive grid layout
   - Implement card design with title, description, tools, and difficulty badge

4. **Footer** (`src/theme/Footer.js`)
   - Update to remove broken links

## Custom Styling

1. **Tailwind Configuration** (`tailwind.config.js`)
   - Add custom colors for Physical AI Book branding
   - Primary: dark navy/charcoal
   - Accent: electric blue/cyan

2. **Custom CSS** (`src/css/custom.css`)
   - Override Docusaurus defaults as needed
   - Ensure responsive design

## Testing

1. **Component Testing**
   - Test each UI component in isolation
   - Verify responsive behavior on different screen sizes

2. **Navigation Testing**
   - Verify all navigation links work correctly
   - Check active page highlighting
   - Test 404 page handling

## Deployment

```bash
# Build the static site
npm run build

# Serve the built site locally for testing
npm run serve
```