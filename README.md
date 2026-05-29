# Autonomy From Scratch

Building autonomous robotics algorithms from first principles using C++ and Webots.

This project documents my journey implementing robotics and control algorithms from scratch, focusing on:

- Control Systems
- Sensor fusion
- Filtering and Signal Conditioning
- Navigation algorithms
- Simulation-based robotics development

All simulations are built and tested in Webots.

---

# Project Roadmap

| Topic               | Status  |
| ------------------- | ------- |
| Line Following      | ✅       |
| PID Control         | ✅       |
| Maze Solving        | 🚧      |
| Path Planning       | Planned |
| SLAM / Localization | Planned |
| Multi-Agent Systems | Planned |

---

# Control Systems

## Basic Line Following

A simple reactive controller using infrared sensors to follow a line.

### Concepts
- Sensor feedback
- Thresholding
- Differential wheel control
- Reactive robotics

![Line Following Demo](https://github.com/B33Boy/autonomy-from-scratch/blob/main/controllers/line_follower/gifs/Line_Follower.gif)

---

# PID Control

Wall-following experiments exploring proportional, integral, and derivative control behavior.

---

## Proportional (P) Control

Exploring how proportional gain affects:

- Responsiveness
- Oscillation
- Steady-state behavior

![P Controller Demo](https://github.com/B33Boy/autonomy-from-scratch/blob/main/controllers/pid_control/gifs/Corridor_P.gif)

### Observations

- Higher proportional gain increases responsiveness
- Excessive gain introduces oscillation
- Sensor noise significantly affects stability

---

## Proportional-Derivative (PD) Control

Adding derivative damping to reduce oscillation and improve stability.

![PD Controller Demo](https://github.com/B33Boy/autonomy-from-scratch/blob/main/controllers/pid_control/gifs/Corridor_PD.gif)

### Concepts

- Damping
- Error rate-of-change
- Stability improvement
- Noise sensitivity

---

## Full PID Control

Introducing integral control to reduce long-term steady-state error.

![PID Controller Demo](https://github.com/B33Boy/autonomy-from-scratch/blob/main/controllers/pid_control/gifs/Corridor_PID.gif)

### Concepts

- Integral accumulation
- Steady-state correction
- Integral windup
- Controller balancing

---

## PID with Low Pass Filtering

Applying filtering techniques to smooth noisy sensor measurements before control computation.

![PID Controller Demo with LPF](https://github.com/B33Boy/autonomy-from-scratch/blob/main/controllers/pid_control/gifs/Corridor_PID_LPF.gif)

### Concepts

- Signal filtering
- Noise reduction
- Sensor smoothing
- Real-world controller robustness

---

# Maze Solving

Work in progress.

Planned topics:

- Graph search
- Wall following
- Flood fill
- Path memory
- Autonomous exploration

---

# Future Goals

- Occupancy grid mapping
- A- path planning
- Particle filters
- Kalman filters
- Visual SLAM
- Multi-robot coordination
- Reinforcement learning experiments

---
