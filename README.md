# MuJoCo Robot Control Infra

An open-source robot teleoperation infra using MuJoCo physics simulation. Control a Franka Emika Panda robotic arm with keyboard to complete various tasks.

## Features

- **MuJoCo Physics Simulation**: High-fidelity physics engine with realistic robot dynamics
- **Keyboard Teleoperation**: Intuitive keyboard controls for 6DOF end-effector motion
- **Task System**: Built-in example task (Close the Box) with extensible architecture
- **Trajectory Recording**: Record and download control trajectories for data collection
- **Frontend-Only Architecture**: No backend server required, works out of the box

## Quick Start

### Prerequisites

- Node.js 18+
- npm or yarn

### Installation

```bash
# Install dependencies
npm install

# Start development server
npm run dev
```

Open [http://localhost:3000](http://localhost:3000) in your browser.

## Keyboard Controls

### Move Gripper

| Key | Action |
|-----|--------|
| `↑` `↓` `←` `→` | Translate in XY plane |
| `E` / `D` | Move up / down |

### Rotate Gripper

| Key | Action |
|-----|--------|
| `Q` / `W` | Roll (positive / negative) |
| `A` / `S` | Pitch (positive / negative) |
| `Z` / `X` | Yaw (positive / negative) |

### Other Actions

| Key | Action |
|-----|--------|
| `Space` | Toggle gripper open/close |
| `R` | Reset scene |

### Camera Controls

- **Left drag** - Orbit camera
- **Right drag** - Pan camera
- **Scroll** - Zoom in/out

## Project Structure

```
.
├── public/
│   ├── data/
│   │   └── tasks.json              # Task definitions
│   └── mujoco-assets/              # MuJoCo model assets (~44MB)
│       ├── scenes/
│       │   └── close_box.xml       # Close the Box task scene
│       ├── includes/
│       │   ├── box_base.xml        # Box model definition
│       │   └── franka_panda.xml    # Franka Panda robot definition
│       └── robots/
│           └── franka_emika_panda/
│               ├── assets/         # 67 obj/stl model files
│               └── LICENSE
├── src/
│   ├── app/
│   │   ├── page.tsx                # Home page (redirects to /tasks)
│   │   ├── tasks/                  # Task list page
│   │   ├── task-running/           # Task execution page (MuJoCo control)
│   │   └── api/tasks/[id]/         # Task data API
│   ├── components/
│   │   ├── GlobalNav.tsx           # Navigation bar
│   │   ├── GlobalFooter.tsx        # Footer
│   │   └── mujoco-framework-next/  # MuJoCo framework
│   │       ├── main.js             # Main module
│   │       ├── mujocoUtils.js      # MuJoCo utilities
│   │       ├── checkers/           # Task completion checkers
│   │       │   ├── CheckerManager.js
│   │       │   ├── BoxJointPositionChecker.js
│   │       │   ├── CompositeChecker.js
│   │       │   └── GripperOpenChecker.js
│   │       └── utils/
│   │           ├── IKController.js         # Inverse kinematics controller
│   │           ├── KeyboardStateManager.js # Keyboard state management
│   │           ├── DragStateManager.js     # Drag state management
│   │           └── Debug.js                # Debug utilities
│   └── lib/
│       ├── local-task-api.ts       # Local task API
│       ├── mujoco-utils.ts         # MuJoCo utilities
│       ├── task-metadata.ts        # Task metadata
│       ├── trajectory-types.ts     # Trajectory type definitions
│       └── use-trajectory-recorder.ts # Trajectory recording hook
├── package.json
├── next.config.ts
└── README.md
```

## Example Task: Close the Box

The project includes a built-in example task: use the robotic arm to push the box lid closed.

**Task Steps:**
1. Move the gripper above the box lid
2. Push the lid down
3. Ensure the box is fully closed

## Adding New Tasks

### 1. Create a Scene File

Create an MJCF XML scene file in `public/mujoco-assets/scenes/`.

### 2. Add Task Definition

Edit `public/data/tasks.json`:

```json
{
  "tasks": [
    {
      "id": 1,
      "name": "Task Name",
      "description": "Task description",
      "difficulty_stars": 2,
      "expected_duration": 300,
      "time_limit": 180,
      "mjcf_xml": "scenes/your_scene.xml",
      "checker_config": {
        "type": "joint_position",
        "joint_name": "body_name/joint_name",
        "target_position": 0.0,
        "tolerance": 0.05,
        "comparison": "less_than"
      },
      "initial_state": {
        "objects": { ... },
        "robots": { ... }
      },
      "steps": [
        { "order": 1, "description": "Step 1" },
        { "order": 2, "description": "Step 2" }
      ]
    }
  ]
}
```

### Checker Configuration

Supported checker types:

| Type | Description | Parameters |
|------|-------------|------------|
| `joint_position` | Check joint position | `joint_name`, `target_position`, `tolerance`, `comparison` |
| `gripper_open` | Check gripper state | `expected_open` |
| `composite` | Combine multiple checkers | `checkers` (array) |

`comparison` options: `less_than`, `greater_than`, `equal` (default)

## Production Build

```bash
npm run build
npm start
```

## Tech Stack

- **Next.js 16** - React full-stack framework
- **React 19** - UI library
- **MuJoCo** - Physics simulation engine
- **Three.js** - 3D rendering
- **Tailwind CSS** - Styling framework
- **TypeScript** - Type safety

## Acknowledgements

We gratefully acknowledge the use of [mujocoWASM](https://github.com/zalo/mujoco_wasm).

## License

MIT License

---

> This is a minimal open-source version focused on core robot teleoperation functionality.
