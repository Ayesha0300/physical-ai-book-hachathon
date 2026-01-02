# Physics Simulation Diagrams

## Diagram 1: Physics Engine Architecture

### Title: Physics Engine Components and Data Flow

### Description:
This diagram shows the main components of a physics engine and how they interact with each other during simulation.

### Elements:
- **Collision Detection System**:
  - Broad phase collision detection
  - Narrow phase collision detection
  - Contact generation

- **Force Application System**:
  - Gravity application
  - External forces
  - Constraint forces

- **Integration System**:
  - State update (position, velocity)
  - Time step management
  - Numerical integration method

- **Constraint Solver**:
  - Joint constraints
  - Contact constraints
  - Iterative solution process

### Key Relationships:
- Input: Object properties (mass, geometry, initial state)
- Process: Time-stepped physics simulation
- Output: Updated object states (position, velocity, forces)

### Caption:
"Figure PS-PE-001: Architecture of a physics engine showing the main components and their interactions during simulation time steps."

---

## Diagram 2: Time Step and Integration Process

### Title: Discrete Time Step Simulation Process

### Description:
This process flow diagram illustrates how physics engines update simulation state through discrete time steps.

### Elements:
- **Start of Time Step** (t = n*dt)
- **Force Calculation**:
  - Gravity forces
  - Applied forces
  - Contact forces
  - Constraint forces

- **Integration Step**:
  - Velocity update: v(t+dt) = v(t) + a*dt
  - Position update: p(t+dt) = p(t) + v(t+dt)*dt

- **Collision Detection**:
  - Broad phase (potential collisions)
  - Narrow phase (precise contact detection)

- **Constraint Resolution**:
  - Solve joint constraints
  - Apply contact forces
  - Update velocities/positions

- **End of Time Step** (t = (n+1)*dt)

### Key Concepts Highlighted:
- Discrete time stepping
- Integration method impact
- Iterative constraint solving
- Sequential process flow

### Caption:
"Figure PS-TS-001: Process flow showing how physics engines update simulation state through discrete time steps, including force calculation, integration, and constraint resolution."

---

## Diagram 3: Rigid Body Properties and Constraints

### Title: Rigid Body Model and Joint Constraints

### Description:
This diagram illustrates the key properties of rigid bodies and the types of constraints that connect them in physics simulation.

### Elements:
- **Rigid Body Components**:
  - Mass and center of mass
  - Inertia tensor
  - Collision geometry
  - Position and orientation

- **Joint Constraint Types**:
  - Revolute joint (single axis rotation)
  - Prismatic joint (linear motion)
  - Ball joint (multi-axis rotation)
  - Fixed joint (no relative motion)

- **Constraint Relationships**:
  - Joint limits
  - Actuator forces/torques
  - Constraint forces

### Key Relationships:
- Rigid body properties determine motion response
- Joint constraints limit relative motion
- Actuators apply forces/torques at joints

### Caption:
"Figure PS-RB-001: Rigid body model showing mass properties, collision geometry, and various joint constraint types used in physics simulation of robotic systems."

---

## Diagram 4: Force Application and Motion

### Title: Physics Simulation Forces and Motion

### Description:
This diagram shows how different types of forces affect rigid body motion in physics simulation.

### Elements:
- **Force Types**:
  - Gravity (constant downward force)
  - Applied forces (user-defined or actuator forces)
  - Contact forces (collision response)
  - Constraint forces (joint limitations)

- **Motion Response**:
  - Linear motion (F = ma)
  - Angular motion (τ = Iα)
  - Combined motion

- **Physics Equations**:
  - Newton's second law
  - Euler's rotation equations
  - Integration over time steps

### Key Concepts Highlighted:
- Force to acceleration relationship
- Linear vs. angular effects
- Integration process
- Multiple force contributions

### Caption:
"Figure PS-FM-001: Force application in physics simulation showing how different force types contribute to linear and angular motion of rigid bodies."