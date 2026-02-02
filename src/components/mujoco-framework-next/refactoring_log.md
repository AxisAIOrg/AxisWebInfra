# MuJoCo Framework 重构文档

## 概述

`main.js` 原本是一个"万能控制器"，将所有功能（资源加载、场景管理、IK控制、轨迹录制、渲染等）堆积在一个文件中，导致代码臃肿、耦合严重、难以维护和测试。

经过重构，`main.js` 现在仅作为**流程编排器**和**状态聚合器**，核心功能已拆分到 `utils/` 目录下的独立模块。

---

## 当前架构

```
main.js (MuJoCoDemo 类)
│
├── 状态聚合：持有运行时所需的所有状态字段
├── 流程编排：串联初始化、渲染、交互等流程
├── 方法委托：大部分方法委托给外部模块实现
│
└── utils/
    ├── assetLoader.js       # 资源加载与 XML 解析
    ├── sceneResolver.js     # 场景加载策略与 VFS 管理
    ├── initRuntime.js       # 初始化流程编排
    ├── rendererRuntime.js   # Three.js 渲染初始化
    ├── teleopController.js  # Teleop/IK/键盘控制
    ├── trajectoryManager.js # 轨迹采样/回放/序列化
    ├── checkerRuntime.js    # Checker 初始化与成功判定
    ├── stateRuntime.js      # 初始状态/随机化
    ├── successTracker.js    # 成功追踪与进度打印
    ├── nameLookup.js        # 名称解析工具
    ├── statusBroadcaster.js # 状态广播（React 集成）
    ├── robot_config.js      # 机器人配置解析
    ├── IKController.js      # IK 控制器实现
    └── KeyboardStateManager.js # 键盘状态管理
```

---

## MuJoCoDemo 类结构

### 1. 状态字段

| 字段 | 说明 |
|------|------|
| `mujoco`, `model`, `data` | MuJoCo 核心对象 |
| `bodies`, `lights` | Three.js 场景对象映射 |
| `scene`, `camera`, `renderer`, `controls` | Three.js 渲染相关 |
| `ikController` | IK 控制器实例 |
| `keyboardStateManager` | 键盘状态管理器 |
| `checkerManager` | Checker 管理器 |
| `robotConfig`, `robotControlConfig` | 机器人配置 |
| `currentEpisode`, `episodes` | 轨迹数据 |
| `replaySamples`, `replayPlaying` | 回放状态 |
| `successAchieved`, `successTime` | 成功判定状态 |
| `initialState`, `checkpointState` | 状态快照 |

### 2. 构造函数

```javascript
constructor(options = {})
```

主要职责：
- 初始化状态字段
- 创建容器元素并挂载到父元素
- 调用 `setupRenderer()` 初始化 Three.js
- 调用 `setupKeyboardControls()` 初始化键盘控制

支持的 options：
- `parentElement` / `parent`: 挂载容器（默认 `document.body`）
- `onLoadingProgress`: 加载进度回调
- `checkerConfig`: Checker 配置
- `initialState`: 初始状态配置
- `domainRandomization`: 域随机化配置

### 3. 核心方法

#### 初始化

| 方法 | 说明 | 委托模块 |
|------|------|----------|
| `init()` | 主初始化入口 | `initRuntime.js` |
| `setupReplay()` | 设置回放模式 | `initRuntime.js` |
| `onModelReloaded()` | 模型重载后回调 | `initRuntime.js` |
| `setupCheckers()` | 初始化 Checker | `checkerRuntime.js` |
| `setupIKController()` | 初始化 IK 控制器 | `teleopController.js` |
| `setupGripperControls()` | 初始化夹爪控制 | `teleopController.js` |
| `setupRobotConfig()` | 解析机器人配置 | `robot_config.js` |

#### 渲染循环

```javascript
render(timeMS)
```

主渲染循环，处理：
1. 回放模式更新 (`updateReplay`)
2. OrbitControls 更新
3. 键盘状态更新
4. IK 控制器更新
5. 物理步进 (`mj_step`)
6. 轨迹采样 (`recordEpisodeSample`)
7. 成功追踪 (`updateSuccessTracking`)
8. Checker 更新 (`updateCheckers`)
9. Three.js 渲染

#### 轨迹管理

| 方法 | 说明 | 委托模块 |
|------|------|----------|
| `recordEpisodeSample()` | 采样当前状态 | `trajectoryManager.js` |
| `saveTrajectory()` | 保存轨迹到服务器 | `trajectoryManager.js` |
| `serializeTrajectory()` | 序列化轨迹 | `trajectoryManager.js` |
| `loadTrajectoryFromData()` | 加载轨迹数据 | `trajectoryManager.js` |
| `startReplayTrajectory()` | 开始回放 | `trajectoryManager.js` |
| `stopReplayTrajectory()` | 停止回放 | `trajectoryManager.js` |
| `updateReplay()` | 更新回放帧 | `trajectoryManager.js` |
| `completeEpisode()` | 完成并导出轨迹 | 内部实现 |

#### Teleop 控制

| 方法 | 说明 | 委托模块 |
|------|------|----------|
| `handleTeleopTranslate()` | 处理平移输入 | `teleopController.js` |
| `handleTeleopRotate()` | 处理旋转输入 | `teleopController.js` |
| `setGripperState()` | 设置夹爪状态 | `teleopController.js` |
| `toggleGripper()` | 切换夹爪状态 | 内部实现 |

#### 状态管理

| 方法 | 说明 | 委托模块 |
|------|------|----------|
| `applyInitialState()` | 应用初始状态 | `stateRuntime.js` |
| `applyDomainRandomization()` | 应用域随机化 | `stateRuntime.js` |
| `captureInitialState()` | 捕获初始状态快照 | 内部实现 |
| `restoreInitialState()` | 恢复初始状态 | 内部实现 |
| `createStateSnapshot()` | 创建状态快照 | 内部实现 |
| `applyStateSnapshot()` | 应用状态快照 | 内部实现 |
| `saveCheckpoint()` | 保存检查点 | 内部实现 |
| `restoreCheckpoint()` | 恢复检查点 | 内部实现 |

#### 名称解析

| 方法 | 说明 | 委托模块 |
|------|------|----------|
| `refreshNameBuffer()` | 刷新名称缓冲区 | `nameLookup.js` |
| `readNameAt()` | 读取指定地址的名称 | `nameLookup.js` |
| `findIdByName()` | 按名称查找 ID | `nameLookup.js` |
| `getJointAddress()` | 获取关节地址 | `nameLookup.js` |

#### React 集成

| 方法 | 说明 | 委托模块 |
|------|------|----------|
| `broadcastStatus()` | 广播状态到 React | `statusBroadcaster.js` |
| `startStatusBroadcasting()` | 开始定期广播 | `statusBroadcaster.js` |
| `stopStatusBroadcasting()` | 停止广播 | `statusBroadcaster.js` |
| `reset()` | 重置仿真 | 内部实现 |
| `fakeDone()` | 触发假完成 | 内部实现 |
| `loadTrajectory()` | 加载轨迹并回放 | 内部实现 |

#### 其他

| 方法 | 说明 |
|------|------|
| `updateSceneFromData()` | 从 MuJoCo 数据更新 Three.js 场景 |
| `onWindowResize()` | 窗口大小变化处理 |
| `resetSimulation()` | 重置仿真状态 |
| `resetEpisode()` | 重置当前轨迹 |
| `setRobotReadyPose()` | 设置机器人准备姿态 |

---

## 模块职责说明

### assetLoader.js
- ZIP 资源加载与解压
- HTTP 资源加载
- XML 引用解析
- VFS 文件写入

### sceneResolver.js
- MuJoCo VFS 初始化
- 任务 XML 拉取（从后端 API）
- 场景加载与 fallback 策略

### initRuntime.js
- `initDemo()`: 主初始化流程
- `setupReplay()`: 回放模式设置
- `onModelReloaded()`: 模型重载处理

### rendererRuntime.js
- Three.js Scene/Camera/Renderer 初始化
- OrbitControls 初始化
- ResizeObserver 设置

### teleopController.js
- IK 控制器初始化
- 夹爪控制逻辑
- 键盘控制回调
- Teleop 平移/旋转处理

### trajectoryManager.js
- 轨迹采样 (`recordEpisodeSample`)
- 轨迹序列化 (`serializeTrajectory`)
- 轨迹保存 (`saveTrajectory`)
- 回放转换与控制

### checkerRuntime.js
- Checker 初始化
- Checker 状态更新
- 成功判定导出

### stateRuntime.js
- 初始状态应用 (`applyInitialState`)
- 域随机化应用 (`applyDomainRandomization`)

### successTracker.js
- 成功判定追踪
- 进度打印
- 调试日志

### nameLookup.js
- 名称缓冲区管理
- 名称查找工具函数

### statusBroadcaster.js
- 状态广播到 React
- 定期广播控制

### robot_config.js
- 机器人配置解析
- 控制参数提取

---

## 数据流

```
用户输入 (键盘/鼠标)
    │
    ▼
KeyboardStateManager ──► teleopController
    │                         │
    │                         ▼
    │                    IKController
    │                         │
    └────────────────────────►│
                              ▼
                    MuJoCo (mj_step)
                              │
                              ▼
                    updateSceneFromData()
                              │
                              ▼
                    Three.js Render
                              │
                              ▼
                    recordEpisodeSample()
                              │
                              ▼
                    checkerRuntime (成功判定)
                              │
                              ▼
                    statusBroadcaster (状态广播)
```

---

## 回调接口

| 回调 | 说明 |
|------|------|
| `onLoadingProgress(update)` | 加载进度通知 |
| `onStatusUpdate(status)` | 状态更新通知 |
| `onTrajectoryExport(trajectory, metadata)` | 轨迹导出通知 |

---

## 设计原则

1. **KISS 原则**：每个模块职责单一，接口精简
2. **功能等价迁移**：重构不改变业务逻辑
3. **最小耦合**：模块间通过参数传递 `demo` 实例，避免全局状态
4. **向后兼容**：保留原有 API，支持渐进式迁移

---

## 后续可优化点

1. **状态模型重构**：当前 `MuJoCoDemo` 仍持有大量状态字段，可考虑引入状态管理器
2. **依赖注入**：减少对 `window`、`document` 等全局对象的依赖
3. **单元测试**：模块化后更容易编写单元测试
4. **TypeScript 迁移**：增强类型安全

---

## 文件结构

```
mujoco-framework-next/
├── main.js                    # 主入口（流程编排）
├── mujocoUtils.js             # MuJoCo 工具函数
├── README.md                  # 项目说明
├── refactoring_log.md         # 本文档
└── utils/
    ├── assetLoader.js         # 资源加载
    ├── checkerRuntime.js      # Checker 运行时
    ├── Debug.js               # 调试工具
    ├── DragStateManager.js    # 拖拽状态管理
    ├── IKController.js        # IK 控制器
    ├── initRuntime.js         # 初始化运行时
    ├── KeyboardStateManager.js# 键盘状态管理
    ├── nameLookup.js          # 名称解析
    ├── Reflector.js           # 反射工具
    ├── rendererRuntime.js     # 渲染运行时
    ├── robot_config.js        # 机器人配置
    ├── sceneResolver.js       # 场景解析
    ├── stateRuntime.js        # 状态运行时
    ├── statusBroadcaster.js   # 状态广播
    ├── successTracker.js      # 成功追踪
    ├── teleopController.js    # Teleop 控制
    └── trajectoryManager.js   # 轨迹管理
```

---

*最后更新：2026-01-18*
