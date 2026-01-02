# Data Model: Physical AI Platform – Landing Page & UI/UX Stabilization

## Module Entity
- **id**: string (unique identifier for the module)
- **title**: string (display title of the module)
- **description**: string (brief description of the module focus)
- **tools**: string[] (key tools used in the module: ROS 2, Gazebo, Isaac, LLMs)
- **difficulty**: string (Beginner/Intermediate/Advanced)
- **route**: string (URL path to the module page, e.g., `/modules/module-x`)

## NavigationItem Entity
- **label**: string (display text for the navigation item)
- **route**: string (URL path for the navigation item)
- **isActive**: boolean (indicates if this is the current page)

## PageLayout Entity
- **navbar**: object (configuration for the navigation bar)
- **content**: object (main content area structure)
- **footer**: object (configuration for the footer)

## VisualIdentity Entity
- **logo**: string (path to SVG logo file)
- **primaryColor**: string (primary color in hex format, e.g., #1e3a8a for dark navy)
- **accentColor**: string (accent color in hex format, e.g., #06b6d4 for electric blue)
- **darkMode**: boolean (whether dark mode is enabled)