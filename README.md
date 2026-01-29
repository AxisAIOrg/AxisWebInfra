<div align="center">

# AxisWebInfra

**An open-source robot teleoperation platform using MuJoCo physics simulation in the broswer!**

[![Next.js](https://img.shields.io/badge/Next.js-16-black?style=flat-square&logo=next.js)](https://nextjs.org/)
[![React](https://img.shields.io/badge/React-19-61DAFB?style=flat-square&logo=react)](https://react.dev/)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.0-3178C6?style=flat-square&logo=typescript)](https://www.typescriptlang.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue?style=flat-square)](LICENSE)

[Features](#features) • [Quick Start](#quick-start) • [Controls](#keyboard-controls) • [Add Tasks](#adding-new-tasks) • [Tech Stack](#tech-stack)

</div>

---

## Overview

Control a **Franka Emika Panda** robotic arm with keyboard inputs to complete various manipulation tasks. This platform provides high-fidelity physics simulation, intuitive teleoperation controls, and trajectory recording capabilities — all running directly in your browser with no backend required.

## Demo Video


https://github.com/user-attachments/assets/6f25324c-f765-45a0-8658-d4e4c4e4ec1e


<div align="center">
<br>

| Simulation | Task Completion |
|:----------:|:---------------:|
| *Physics-based robot control* | *Goal-oriented manipulation tasks* |

</div>

---

## Features

<table>
<tr>
<td width="50%">

### Core Capabilities

- **MuJoCo Physics Engine**  
  High-fidelity simulation with realistic robot dynamics and contact physics

- **6DOF Teleoperation**  
  Intuitive keyboard controls for full end-effector motion control

- **Task Framework**  
  Extensible architecture with built-in task completion verification

</td>
<td width="50%">

### Developer Features

- **Trajectory Recording**  
  Capture and export control data for learning and analysis

- **Frontend-Only Architecture**  
  Zero backend dependencies — works out of the box

- **Modular Design**  
  Easy to extend with new tasks, checkers, and robot models

</td>
</tr>
</table>

---

## Quick Start

### Prerequisites

| Requirement | Version |
|-------------|---------|
| Node.js | 18+ |
| npm / yarn | Latest |

### Installation

```bash
# Clone the repository
git clone https://github.com/your-username/axis-mujoco-infra.git
cd axis-mujoco-infra

# Install dependencies
npm install

# Start development server
npm run dev
```

Open **[http://localhost:3000](http://localhost:3000)** in your browser.

### Production Build

```bash
npm run build
npm start
```

---

## Keyboard Controls

### Movement

<table>
<tr>
<th align="center">Translation</th>
<th align="center">Vertical</th>
</tr>
<tr>
<td>

| Key | Action |
|:---:|--------|
| `↑` | Forward (+Y) |
| `↓` | Backward (-Y) |
| `←` | Left (-X) |
| `→` | Right (+X) |

</td>
<td>

| Key | Action |
|:---:|--------|
| `E` | Move Up (+Z) |
| `D` | Move Down (-Z) |

</td>
</tr>
</table>

### Rotation

| Key | Axis | Direction |
|:---:|:----:|-----------|
| `Q` / `W` | Roll | Positive / Negative |
| `A` / `S` | Pitch | Positive / Negative |
| `Z` / `X` | Yaw | Positive / Negative |

### Actions & Camera

<table>
<tr>
<td>

**Robot Actions**
| Key | Action |
|:---:|--------|
| `Space` | Toggle Gripper |
| `R` | Reset Scene |

</td>
<td>

**Camera Controls**
| Input | Action |
|:-----:|--------|
| Left Drag | Orbit |
| Right Drag | Pan |
| Scroll | Zoom |

</td>
</tr>
</table>

---

## Example Task: Close the Box

The platform includes a demonstration task where the robot must push a box lid closed.

**Objective:** Use the gripper to push the hinged lid until the box is fully closed.

| Step | Action |
|:----:|--------|
| 1 | Position gripper above the box lid |
| 2 | Apply downward pressure to close the lid |
| 3 | Verify complete closure (automatic detection) |

---

## Adding New Tasks

### Step 1: Create Scene File

Create an MJCF XML scene in `public/mujoco-assets/scenes/`:

```xml
<!-- your_scene.xml -->
<mujoco>
  <include file="../includes/franka_panda.xml"/>
  <!-- Add your objects here -->
</mujoco>
```

### Step 2: Register Task

Add your task definition to `public/data/tasks.json`:

```json
{
  "id": 2,
  "name": "Your Task Name",
  "description": "Brief description of the task objective",
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
  "steps": [
    { "order": 1, "description": "First step" },
    { "order": 2, "description": "Second step" }
  ]
}
```

### Checker Types

| Type | Description | Parameters |
|------|-------------|------------|
| `joint_position` | Verify joint reaches target | `joint_name`, `target_position`, `tolerance`, `comparison` |
| `gripper_open` | Check gripper state | `expected_open` (boolean) |
| `composite` | Combine multiple checkers | `checkers` (array of checker configs) |

> **Comparison Options:** `less_than`, `greater_than`, `equal` (default)

---

## Project Structure

```
├── public/
│   ├── data/
│   │   └── tasks.json                 # Task definitions
│   └── mujoco-assets/                 # MuJoCo models (~44MB)
│       ├── scenes/                    # Task scene files
│       ├── includes/                  # Shared model components
│       └── robots/franka_emika_panda/ # Robot meshes (67 files)
│
├── src/
│   ├── app/
│   │   ├── page.tsx                   # Home (redirects to /tasks)
│   │   ├── tasks/                     # Task selection page
│   │   ├── task-running/              # MuJoCo control interface
│   │   └── api/tasks/[id]/            # Task data endpoint
│   │
│   ├── components/
│   │   ├── GlobalNav.tsx              # Navigation
│   │   ├── GlobalFooter.tsx           # Footer
│   │   └── mujoco-framework-next/     # Core simulation framework
│   │       ├── main.js                # Entry point
│   │       ├── mujocoUtils.js         # MuJoCo helpers
│   │       ├── checkers/              # Task completion logic
│   │       └── utils/                 # IK, keyboard, debug utilities
│   │
│   └── lib/
│       ├── task-metadata.ts           # Task type definitions
│       ├── trajectory-types.ts        # Recording data structures
│       └── use-trajectory-recorder.ts # Recording React hook
│
└── package.json
```

---

## Tech Stack

<table>
<tr>
<td align="center" width="100">
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/nextjs/nextjs-original.svg" width="48" height="48" alt="Next.js" />
<br><strong>Next.js 16</strong>
</td>
<td align="center" width="100">
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/react/react-original.svg" width="48" height="48" alt="React" />
<br><strong>React 19</strong>
</td>
<td align="center" width="100">
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/typescript/typescript-original.svg" width="48" height="48" alt="TypeScript" />
<br><strong>TypeScript</strong>
</td>
<td align="center" width="100">
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon/icons/threejs/threejs-original.svg" width="48" height="48" alt="Three.js" />
<br><strong>Three.js</strong>
</td>
<td align="center" width="100">
<img src="https://cdn.jsdelivr.net/gh/devicons/devicon@latest/icons/tailwindcss/tailwindcss-original.svg" width="48" height="48" alt="Tailwind" />
<br><strong>Tailwind CSS</strong>
</td>
</tr>
</table>

| Component | Technology |
|-----------|------------|
| Framework | Next.js 16 (App Router) |
| UI Library | React 19 |
| Physics Engine | MuJoCo (via WASM) |
| 3D Rendering | Three.js |
| Styling | Tailwind CSS |
| Language | TypeScript |

---

## Acknowledgements

This project builds upon the excellent work of the open-source community:

- **[MuJoCo](https://mujoco.org/)** — Physics simulation engine by DeepMind
- **[mujoco_wasm](https://github.com/zalo/mujoco_wasm)** — WebAssembly port enabling browser-based simulation
- **[Franka Emika](https://www.franka.de/)** — Panda robot model and specifications

---

## License

This project is licensed under the **Apache License 2.0** — see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**[Back to Top](#axiswebinfra)**

<sub>Built for robotics research and education</sub>

</div>
