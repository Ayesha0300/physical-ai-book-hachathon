# Physical AI & Humanoid Robotics Book

This repository contains the source code and content for the "Physical AI & Humanoid Robotics" educational book, built with Docusaurus.

## About

This comprehensive book covers the fundamentals of Physical AI and humanoid robotics, including:
- ROS 2 as the middleware nervous system of robots
- Isaac Sim and Gazebo simulation platforms
- Physical AI concepts and machine learning for robots
- Humanoid robot control systems
- Integration and deployment strategies

## Getting Started

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/physical-ai-book.git
   cd physical-ai-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

### Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

### Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

### Deployment

Using SSH:
```bash
USE_SSH=true npm run deploy
```

Not using SSH:
```bash
GH_TOKEN=<GITHUB_TOKEN> npm run deploy
```

For more details on deployment, see the [Docusaurus deployment guide](https://docusaurus.io/docs/deployment).

## Contributing

This book is built using Spec-Kit Plus methodology with Claude Code for AI-assisted content generation. All content follows a spec-driven development approach.

## License

This project is licensed under the MIT License.