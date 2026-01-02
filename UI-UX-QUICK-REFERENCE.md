# UI/UX Quick Reference Guide

## Quick Links to Key Files

### Configuration
- **Main Config**: `C:\Users\DELL XPS\physical-ai-book\docusaurus.config.js`
- **Global Styles**: `C:\Users\DELL XPS\physical-ai-book\src\css\custom.css`
- **Homepage Styles**: `C:\Users\DELL XPS\physical-ai-book\src\pages\index.module.css`

### Components
- **Homepage**: `C:\Users\DELL XPS\physical-ai-book\src\pages\index.js`
- **Navbar**: `C:\Users\DELL XPS\physical-ai-book\src\theme\Navbar.js`

---

## CSS Class Reference

### Accessibility Classes
```css
.skip-to-content          /* Skip navigation link */
*:focus-visible           /* Enhanced focus states */
```

### Loading States
```css
.loading-spinner          /* Default spinner (2rem) */
.loading-spinner--small   /* Small spinner (1rem) */
.loading-spinner--large   /* Large spinner (3rem) */
.skeleton                 /* Skeleton loader base */
.skeleton-text            /* Text skeleton */
.skeleton-heading         /* Heading skeleton */
.skeleton-card            /* Card skeleton */
```

### Animation Classes
```css
.fade-in                  /* Fade in animation */
.animate-on-scroll        /* Scroll-triggered animation */
.scroll-optimized         /* Performance-optimized scrolling */
```

### Component Classes (index.module.css)
```css
.heroBanner               /* Hero section container */
.heroTitle                /* Hero title with gradient */
.gradientText             /* Gradient text effect */
.featureCard              /* Feature card with hover */
.moduleCard               /* Module card with advanced effects */
.ctaSection               /* Call-to-action section */
```

---

## Design Tokens

### Colors
```css
/* Primary Brand */
--ifm-color-primary: #06b6d4         /* Cyan */
--accent-purple: #8b5cf6              /* Purple */
--accent-cyan: #06b6d4                /* Cyan accent */

/* Technology Colors */
--tech-ros: #ff6600                   /* ROS 2 */
--tech-gazebo: #2962ff                /* Gazebo */
--tech-isaac: #7c4dff                 /* Isaac */
--tech-vla: #0d9488                   /* VLA */

/* Semantic Colors */
--accent-success: #10b981             /* Success/Green */
--accent-warning: #f59e0b             /* Warning/Orange */
--accent-error: #ef4444               /* Error/Red */
```

### Spacing
```css
--ifm-spacing-horizontal: 1.5rem
--ifm-spacing-vertical: 1.5rem
--ifm-navbar-height: 4rem
--doc-sidebar-width: 300px
```

### Border Radius
```css
--ifm-code-border-radius: 0.375rem
--ifm-button-border-radius: 0.5rem
--ifm-card-border-radius: 0.75rem
--ifm-global-radius: 0.5rem
```

### Typography
```css
--ifm-font-family-base: 'Inter', sans-serif
--ifm-font-size-base: 16px
--ifm-line-height-base: 1.75
```

### Transitions
```css
--ifm-transition-fast: 150ms ease-in-out
--ifm-transition-default: 250ms ease-in-out
--ifm-transition-slow: 350ms ease-in-out
```

---

## Responsive Breakpoints

```css
/* Mobile */
@media (max-width: 768px) { }

/* Tablet */
@media (min-width: 768px) and (max-width: 996px) { }

/* Mobile + Tablet */
@media (max-width: 996px) { }

/* Desktop */
@media (min-width: 996px) { }
```

---

## Animation Reference

### Keyframe Animations
```css
@keyframes spin                /* Loading spinner rotation */
@keyframes slideInRight        /* Mobile menu entrance */
@keyframes fadeInUp            /* Menu item stagger */
@keyframes skeleton-loading    /* Skeleton shimmer */
@keyframes fadeIn              /* Content fade in */
@keyframes float               /* Hero floating cards */
```

### Easing Functions
```css
cubic-bezier(0.4, 0, 0.2, 1)   /* Standard easing */
cubic-bezier(0.34, 1.56, 0.64, 1) /* Bounce easing */
```

---

## Component Hover Effects

### Module Cards
```css
/* Hover State */
transform: translateY(-8px) scale(1.02)
box-shadow: layered shadows with color
border-color: primary
gradient top border reveal
radial glow effect expansion
```

### Feature Cards
```css
/* Hover State */
transform: translateY(-6px)
box-shadow: elevated
icon: scale(1.15) rotate(5deg)
```

### Buttons
```css
/* Hover State */
transform: translateY(-1px) or translateY(-2px)
box-shadow: elevated
background: darker/lighter
```

---

## Accessibility Quick Tips

### Focus States
- All interactive elements have 3px outline
- Outline offset of 3px for breathing room
- Respects `:focus-visible` for keyboard-only focus

### Touch Targets
- Minimum 44x44px on mobile
- Applies to buttons, links, menu items
- Centered content with flexbox

### Skip Navigation
- Hidden by default (left: -9999px)
- Visible on keyboard focus
- Positioned at top center
- High z-index (9999)

### Motion Preferences
```css
@media (prefers-reduced-motion: reduce) {
  /* All animations reduced to 0.01ms */
  /* Scroll behavior set to auto */
}
```

---

## Common Tasks

### Change Primary Color
Edit in `src/css/custom.css`:
```css
:root {
  --ifm-color-primary: #YOUR_COLOR;
  /* Update derived colors */
  --ifm-color-primary-dark: ...;
  --ifm-color-primary-light: ...;
}
```

### Add New Animation
1. Define keyframe in `src/css/custom.css`:
```css
@keyframes yourAnimation {
  from { /* start state */ }
  to { /* end state */ }
}
```

2. Apply to element:
```css
.your-element {
  animation: yourAnimation 0.5s ease-in-out;
}
```

### Create Loading State
```jsx
<div className="loading-spinner" role="status" aria-label="Loading">
  <span className="sr-only">Loading...</span>
</div>
```

### Add Skeleton Loader
```jsx
<div className="skeleton skeleton-card" aria-busy="true">
  <div className="skeleton skeleton-heading"></div>
  <div className="skeleton skeleton-text"></div>
  <div className="skeleton skeleton-text"></div>
</div>
```

---

## Navigation Structure

### Navbar Links
```
Home (/)
Modules (/modules)
Capstone (/docs/capstone)
Hardware (/docs/hardware)
GitHub (external)
```

### Footer Links
```
Modules:
  - All Modules (/modules)
  - Module 1-4 (specific paths)
  - Capstone Project

Resources:
  - Get Started (/docs/intro)
  - Hardware Setup (/docs/hardware)
  - Glossary (/docs/glossary)
  - GitHub (external)

Community:
  - License (external)
  - Contribute (external)
  - Discussions (external)
```

---

## Performance Checklist

### Font Loading
- [x] Preconnect to Google Fonts
- [x] display=swap parameter
- [x] Optimized font weights only

### CSS Performance
- [x] will-change on animated elements
- [x] Transform and opacity for animations
- [x] Avoid animating layout properties

### Images
- [ ] TODO: Add next-gen formats (WebP)
- [ ] TODO: Implement lazy loading
- [ ] TODO: Optimize image sizes

---

## Testing Checklist

### Manual Testing
- [ ] Test all navigation links
- [ ] Verify dark mode styling
- [ ] Test keyboard navigation (Tab, Enter, Escape)
- [ ] Check mobile menu on phone
- [ ] Test hover states on desktop
- [ ] Verify loading spinners
- [ ] Test form inputs if applicable

### Accessibility Testing
- [ ] Run Lighthouse audit (score > 90)
- [ ] Test with WAVE extension
- [ ] Verify color contrast ratios
- [ ] Test with screen reader
- [ ] Keyboard-only navigation test

### Browser Testing
- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest)
- [ ] Mobile Safari
- [ ] Edge (latest)

---

## Troubleshooting

### Build Fails
1. Check CSS syntax in `.module.css` files
2. Verify all imports are correct
3. Clear `.docusaurus` cache: `npm run clear`
4. Reinstall dependencies: `npm ci`

### Styles Not Applying
1. Check CSS specificity
2. Verify dark mode selectors: `[data-theme='dark']`
3. Clear browser cache
4. Check for typos in class names

### Animations Janky
1. Use transform and opacity only
2. Add `will-change: transform` sparingly
3. Reduce animation complexity
4. Test on lower-end devices

### Links Broken
1. Use relative paths for internal links
2. Verify file structure matches routes
3. Check for README.md redirects
4. Use browser DevTools Network tab

---

## Resources

### Documentation
- [Docusaurus Docs](https://docusaurus.io)
- [Tailwind CSS Docs](https://tailwindcss.com)
- [MDN Web Docs](https://developer.mozilla.org)

### Accessibility
- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [A11y Project](https://www.a11yproject.com)
- [WebAIM](https://webaim.org)

### Tools
- [Chrome DevTools](https://developer.chrome.com/docs/devtools/)
- [Lighthouse](https://developers.google.com/web/tools/lighthouse)
- [WAVE Extension](https://wave.webaim.org/extension/)

---

## Version History

**v1.0.0** (2026-01-01)
- Initial comprehensive UI/UX improvements
- Navigation fixes
- Accessibility enhancements
- Mobile responsiveness
- Micro-interactions
- Performance optimizations

---

For detailed implementation notes, see: `UI-UX-IMPROVEMENTS.md`
