# AI Robot Arena

A 2D autonomous robot navigation simulation developed in **C++**, using **SDL3**, **Box2D**, and the **A* pathfinding algorithm**.

The project demonstrates intelligent path planning, physics-based movement, collision handling, dynamic obstacle avoidance, and switching between manual and autonomous control.

## Features

- **A* Pathfinding:** Calculates a route from the robot's position to the goal.
- **Autonomous Navigation:** The robot follows waypoints along the calculated path.
- **Box2D Physics:** Handles robot movement and collisions with obstacles.
- **Dynamic Obstacles:** Allows an obstacle to be added during gameplay, triggering path recalculation.
- **Manual and AI Modes:** Switch between keyboard control and autonomous navigation.
- **Intelligent Path Rejoining:** Continues along an existing path when the robot is on it; otherwise, calculates a new path.
- **Stuck Detection:** Detects when the robot stops making progress and attempts to calculate an alternative route.
- **Goal Detection:** Displays a success message when the robot reaches the goal.

## Technologies

- C++17 or later
- SDL3 — Rendering, window management, and keyboard input
- Box2D 3.x — 2D physics and collision simulation
- A* — Grid-based pathfinding
- CMake — Build configuration

## Controls

| Key | Action |
|---|---|
| W / A / S / D | Manual robot movement |
| Arrow keys | Manual robot movement |
| M | Activate Manual Mode |
| P | Activate AI Mode |
| O | Add a dynamic obstacle |
| R | Reset the robot and activate AI Mode, retaining obstacles |
| Esc | Exit the application |

## How It Works

The A* algorithm searches a two-dimensional grid to calculate a route around obstacles. The resulting waypoints are displayed as yellow markers.

The robot uses Box2D velocity to move between waypoints, while SDL3 renders the arena, robot, obstacles, and goal.

When the robot moves away from the existing path during manual control, switching back to AI Mode triggers path recalculation. If the robot remains on the existing path, it resumes following that route.

## Project Structure

```text
AI-Robot-Arena/
├── src/
│   └── main.cpp
├── CMakeLists.txt
├── .gitignore
└── README.md
```

## Future Enhancements

- Virtual distance sensors using Box2D ray casting
- Improved real-time obstacle avoidance
- Multiple autonomous robots
- Additional arena layouts
- Performance metrics for pathfinding and navigation

## Author

Haissam Badih

Developed as a C++ game development and autonomous navigation project.