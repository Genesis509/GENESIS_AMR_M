# GENESIS_AMR_M

An autonomous mobile robot with a 5-DOF manipulator, designed as a comprehensive exploration of core robotics concepts. This project implements everything from low-level register programming to high-level vision-language-action (VLA) systems.

![[images/AMR_M.png]]
## **Current Capabilities**

### Mobile Platform

![[images/AMR3.jpg|500]]

- **Teleoperation**: Omnidirectional movement with Xbox controller, live video feedback via MediaMTX, and pan-tilt first-person view control
- **"Good Boy" Mode**: Autonomous search and retrieval of a green ball using omnidirectional movement and pan-tilt camera tracking

### Manipulator

![[images/5DOF_MANIPULATOR.jpg]]

- **Servo-to-Joint Space Mapping**: Translates and constrains commands from joint space to servo space, handling centering, range limits, and register communication
- **Forward Kinematics**: Complete kinematic chain implementation
- **Inverse Kinematics**: Analytical or numerical IK solver for end-effector positioning
- **Blind Pick-and-Place**: Path interpolation for automated manipulation tasks
- **GUI Control & Visualization**: Real-time workspace visualization with joint control, pose goals, and mathematical space representation

## **In Development (Coming Soon)**

### Mobile Platform

- **Autonomous Mapping**: Frontier exploration with A* or RRT* path planning (toggling based on environment structure; RRT* offers faster performance but struggles in tight spaces). *Simulation complete, implementation in progress.*
- **Autonomous Navigation + Obstacle Avoidance**: A* for global path planning with RRT* for dynamic replanning around static obstacles. *Simulation complete, implementation in progress.*
- **Ball Pose Estimation**: Enhanced "Good Boy" mode with 3D position estimation

### Manipulator

- **Image-Based Visual Servoing (IBVS)**: Vision-guided manipulation control

## **Future Roadmap**

- **Integrated Mobile Manipulation**: Full coordination of mobile base and manipulator for pick-and-place tasks using classical computer vision
- **ML-Based Perception**: Object segmentation and classification using machine learning for advanced pick-and-place operations
- **Learning-Based Control**: Machine learning and reinforcement learning approaches for dexterous manipulation

---

_This project represents a complete robotics system built from first principles, providing hands-on experience with embedded systems, kinematics, path planning, Machine vision, and autonomous behaviors._