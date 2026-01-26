# MuJoCo WASM Integration - Architecture and Implementation

This document explains the architecture of MuJoCo WASM and how the Put Banana Task test page was implemented.

---

## Part 1: WASM and XML Architecture

### Key Concept: Two Different Components

There are **two completely different components** in MuJoCo:

1. **MuJoCo WASM** = The **physics engine** (compiled code)
2. **MuJoCo XML/MJCF** = The **scene/model data** (description files)

They serve different purposes and cannot be interchanged.

### 1. MuJoCo WASM (The Engine)

#### What It Is
- **Compiled WebAssembly binary** - The physics simulation engine itself
- **Code that runs** - Contains all the physics calculations, collision detection, etc.
- **Already compiled** - Built from C/C++ source code using Emscripten

#### File Format
- **`.wasm`** - Binary WebAssembly format (or embedded in `.js` file)
- **Location**: `public/mujoco-js/dist/mujoco_wasm.js` (contains embedded WASM)

#### What It Does
- Runs physics calculations
- Handles collision detection
- Manages simulation state
- Provides API functions like `mj_step()`, `mj_forward()`, etc.

#### Analogy
Think of it like a **game engine** (Unity, Unreal) - it's the code that makes things work, but it doesn't contain your game levels.

### 2. MuJoCo XML/MJCF (The Scene Data)

#### What It Is
- **Scene description files** - Define what objects, robots, and environments exist
- **Data, not code** - Describes geometry, positions, materials, joints, etc.
- **Loaded at runtime** - The WASM engine reads these files to create the simulation

#### File Format
- **`.xml` or `.mjcf`** - XML-based format (MJCF = MuJoCo XML Format)
- **Human-readable** - You can edit these files to change scenes

#### What It Contains
```xml
<mujoco>
  <worldbody>
    <body name="banana" pos="0.28 -0.58 0.825">
      <joint type="free"/>
      <geom type="capsule" size="0.02 0.15" rgba="1.0 0.9 0.0 1"/>
    </body>
  </worldbody>
</mujoco>
```

#### Analogy
Think of it like a **level file** or **3D model file** - it describes what's in the scene, but the engine (WASM) interprets it.

### How They Work Together

```
┌─────────────────────────────────────────┐
│  MuJoCo WASM (Engine)                   │
│  - Physics calculations                 │
│  - Collision detection                  │
│  - Simulation loop                      │
│  Location: mujoco_wasm.js               │
└──────────────┬──────────────────────────┘
               │
               │ Loads and interprets
               │
               ▼
┌─────────────────────────────────────────┐
│  XML/MJCF Files (Scene Data)            │
│  - Object definitions                   │
│  - Positions and rotations              │
│  - Joint configurations                 │
│  - Material properties                  │
│  Location: Various XML files            │
└─────────────────────────────────────────┘
```

### Example Flow

1. **Load WASM Engine**:
   ```typescript
   const mujoco = await loadMujoco(); // Loads mujoco_wasm.js
   ```

2. **Load Scene XML**:
   ```typescript
   mujoco.FS.writeFile("/working/model.xml", xmlContent);
   const model = mujoco.MjModel.loadFromXML("/working/model.xml");
   ```

3. **Run Simulation**:
   ```typescript
   mujoco.mj_step(model, data); // Engine processes the scene
   ```

### Key Takeaways

- **WASM** = The engine (already exists, compiled from C++)
- **XML** = The scenes (what you load into the engine)
- **Tasks are data, not code** - They cannot be compiled to WASM
- **Architecture**: `WASM Engine (code) → Loads → XML Files (data) → Creates → Simulation`

---

## Part 2: Put Banana Task Implementation

### Overview

The Put Banana Task test page (`frontend/src/app/mujoco/page.tsx`) implements a simplified version of the RoboVerse "put banana" task, where a robot must pick up a banana and place it inside a mug.

### Architecture

```
┌─────────────────────────────────────────┐
│  React Component (page.tsx)            │
│  - State management                     │
│  - UI rendering                         │
│  - Event handling                       │
└──────────────┬──────────────────────────┘
               │
               │ Uses
               ▼
┌─────────────────────────────────────────┐
│  MuJoCo Utilities (mujoco-utils.ts)    │
│  - loadMujoco()                         │
│  - PUT_BANANA_TASK_XML                  │
│  - Coordinate conversion helpers        │
└──────────────┬──────────────────────────┘
               │
               │ Loads
               ▼
┌─────────────────────────────────────────┐
│  MuJoCo WASM Engine                     │
│  - Physics simulation                   │
│  - Model/data management                │
└─────────────────────────────────────────┘
```

### 1. Task XML Definition

The task is defined in `PUT_BANANA_TASK_XML` in `mujoco-utils.ts`:

#### Objects
- **Table**: Fixed box at position `[0.4, -0.2, 0.4]`
- **Banana**: Movable capsule (yellow) at `[0.28, -0.58, 0.825]` with free joint
- **Mug**: Target cylinder (blue) at `[0.48, -0.54, 0.863]` with free joint
- **Other objects**: Book, lamp, remote control, Rubik's cube, vase (decorative)

#### Robot
- **Franka Panda**: Simplified 7-DOF arm with 2-finger gripper
- **Base position**: `[0.89, -0.25, 0.78]`
- **Base rotation**: `[-0.029191, -0.024987, 0.000730, -0.999261]`
- **Joints**: 7 arm joints (hinge) + 2 gripper joints (slide)

#### Actuators
All 9 joints have motor actuators for control:
```xml
<actuator>
  <motor name="panda_joint1_motor" joint="panda_joint1" ... />
  <!-- ... 8 more actuators ... -->
</actuator>
```

### 2. Initialization Flow

```typescript
// Step 1: Load MuJoCo WASM module
const mujoco = await loadMujoco();

// Step 2: Set up virtual file system
mujoco.FS.mkdir("/working");
mujoco.FS.mount(mujoco.MEMFS, { root: "." }, "/working");

// Step 3: Write XML to virtual file system
mujoco.FS.writeFile("/working/model.xml", PUT_BANANA_TASK_XML);

// Step 4: Load model from XML
const model = mujoco.MjModel.loadFromXML("/working/model.xml");

// Step 5: Create simulation data
const data = new mujoco.MjData(model);

// Step 6: Set initial robot joint positions
const initialJointPositions = {
  panda_joint1: -0.000000,
  panda_joint2: -0.785398,
  // ... etc
};
// Apply to data.qpos

// Step 7: Find robot actuators
// Map joint names to actuator indices for control
```

### 3. Robot Control Implementation

#### State Management
```typescript
// Robot joint control state
const robotJointIndicesRef = useRef<Record<string, number>>({});
const robotControlRef = useRef<Record<string, number>>({
  panda_joint1: 0,
  panda_joint2: 0,
  // ... all 9 joints
});
```

#### Actuator Discovery
```typescript
// Find actuators by name
for (const jointName of jointNames) {
  const jointId = mujoco.mj_name2id(model, 3, jointName); // 3 = mjOBJ_JOINT
  for (let a = 0; a < model.nu; a++) {
    const actuatorName = mujoco.mj_id2name(model, 19, a); // 19 = mjOBJ_ACTUATOR
    if (actuatorName && actuatorName.includes(jointName)) {
      robotJointIndicesRef.current[jointName] = a;
      break;
    }
  }
}
```

#### Keyboard Control Mapping
```typescript
// Joint control (position-based)
Q/A → panda_joint1 (base rotation)
W/S → panda_joint2 (shoulder)
E/D → panda_joint3 (elbow)
R/F → panda_joint4
T/G → panda_joint5
Y/H → panda_joint6
U/J → panda_joint7 (wrist)
O/L → Gripper open/close
```

#### Control Application
```typescript
// In animation loop, before mj_step():
for (const [jointName, actuatorIndex] of Object.entries(robotJointIndicesRef.current)) {
  if (actuatorIndex >= 0 && actuatorIndex < model.nu) {
    const controlValue = robotControlRef.current[jointName] || 0;
    data.ctrl[actuatorIndex] = controlValue;
  }
}

mujoco.mj_step(model, data);
```

### 4. Goal Detection

#### Implementation
```typescript
function checkGoal(model: any, data: any) {
  const bananaId = bananaBodyIdRef.current;
  const mugId = mugBodyIdRef.current;
  
  // Get positions in MuJoCo coordinates
  const bananaPos = [
    data.xpos[bananaId * 3 + 0],
    data.xpos[bananaId * 3 + 1],
    data.xpos[bananaId * 3 + 2],
  ];
  const mugPos = [
    data.xpos[mugId * 3 + 0],
    data.xpos[mugId * 3 + 1],
    data.xpos[mugId * 3 + 2],
  ];
  
  // Calculate relative position
  const relativePos = [
    bananaPos[0] - mugPos[0],
    bananaPos[1] - mugPos[1],
    bananaPos[2] - mugPos[2],
  ];
  
  // Check bounds (from RoboVerse checker)
  const lower = [-0.04, -0.04, -0.03];
  const upper = [0.04, 0.04, 0.06];
  
  const inBounds =
    relativePos[0] >= lower[0] && relativePos[0] <= upper[0] &&
    relativePos[1] >= lower[1] && relativePos[1] <= upper[1] &&
    relativePos[2] >= lower[2] && relativePos[2] <= upper[2];
  
  if (inBounds && !goalAchieved) {
    setGoalAchieved(true);
    addLog("🎉 GOAL ACHIEVED! Banana is inside the mug!");
  }
}
```

### 5. Rendering with Three.js

#### Coordinate Conversion
MuJoCo uses Y-up, Three.js uses Z-up. Conversion functions:

```typescript
// Position: MuJoCo (X, Y, Z) → Three.js (X, Z, -Y)
function getPos(buffer, index) {
  return [
    buffer[index * 3 + 0],  // X
    buffer[index * 3 + 2],  // Z (Y and Z swapped)
    -buffer[index * 3 + 1], // -Y
  ];
}

// Quaternion: MuJoCo → Three.js
function getQuat(buffer, index) {
  return [
    -buffer[index * 4 + 1],
    -buffer[index * 4 + 3],
    buffer[index * 4 + 2],
    -buffer[index * 4 + 0],
  ];
}
```

#### Body Rendering
```typescript
// Update body positions every frame
for (let b = 0; b < model.nbody; b++) {
  if (bodies[b]) {
    const pos = getPos(data.xpos, b);
    bodies[b].position.set(pos[0], pos[1], pos[2]);
    
    const quat = getQuat(data.xquat, b);
    bodies[b].quaternion.set(quat[0], quat[1], quat[2], quat[3]);
  }
}
```

### 6. Key Implementation Details

#### Virtual File System
MuJoCo WASM uses Emscripten's virtual file system (MEMFS) to load files:
```typescript
mujoco.FS.mkdir("/working");
mujoco.FS.mount(mujoco.MEMFS, { root: "." }, "/working");
mujoco.FS.writeFile("/working/model.xml", xmlContent);
```

#### Initial State Setting
Robot joint positions are set programmatically after model creation:
```typescript
// Find qpos index for each joint
let qposIndex = 0;
for (let j = 0; j < jointId; j++) {
  const jointType = model.jnt_type[j];
  if (jointType === 0) qposIndex += 7; // free joint
  else if (jointType === 3) qposIndex += 1; // hinge joint
  // ... etc
}
data.qpos[qposIndex] = initialPosition;
```

#### Simulation Loop
```typescript
function animate(timeMS: number) {
  // Apply robot controls
  for (const [jointName, actuatorIndex] of Object.entries(robotJointIndicesRef.current)) {
    data.ctrl[actuatorIndex] = robotControlRef.current[jointName];
  }
  
  // Step physics
  mujoco.mj_step(model, data);
  
  // Update Three.js objects
  for (let b = 0; b < model.nbody; b++) {
    // Update positions/rotations
  }
  
  // Check goal
  checkGoal(model, data);
  
  // Render
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
```

### 7. Current Limitations

1. **Mesh Support**: Objects use primitive geometries (box, cylinder, capsule) instead of actual mesh files
2. **Simplified Robot**: Robot is a simplified representation, not the full mesh-based model
3. **No Grasping Logic**: Robot can collide with banana but doesn't explicitly "grasp" it
4. **Manual Control**: User controls joints directly, not end-effector position

### 8. File Structure

```
frontend/src/app/mujoco/
├── page.tsx              # Main test page component
└── README.md             # This file

frontend/src/lib/
└── mujoco-utils.ts      # MuJoCo utilities and PUT_BANANA_TASK_XML

frontend/public/mujoco-js/dist/
├── mujoco_wasm.js        # MuJoCo WASM engine
└── mujoco_wasm.d.ts      # TypeScript definitions
```

---

## Summary

### Architecture
- **WASM Engine**: Pre-compiled physics engine loaded at runtime
- **XML Files**: Scene descriptions loaded into the engine
- **Separation**: Engine (code) vs. Scenes (data)

### Implementation
- **Task Definition**: XML string with objects and robot
- **Robot Control**: Keyboard → Joint positions → Actuators → Physics
- **Goal Detection**: Position-based checker comparing banana to mug
- **Rendering**: Three.js with coordinate conversion

The implementation demonstrates how to:
1. Load MuJoCo WASM in a Next.js/React application
2. Define a task using XML
3. Control a robot through actuators
4. Detect task completion
5. Render the simulation in real-time

