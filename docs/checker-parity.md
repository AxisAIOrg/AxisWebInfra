# Checker Parity Matrix — JS vs Python vs Hub

> 用途：作为一个 PR 提交到 `AxisAIOrg/AxisWebInfra`（docs），让三个仓库的 checker 状态透明，防止后续 drift。

## 背景

Axis 有三个地方定义任务成功判定（checker）：
- **Hub 后端**（生产环境，真实跑任务）
- **`AxisWebInfra` JS 端**（浏览器内遥操）
- **`AxisDataCleaning` Python 端**（离线轨迹验证）

这三者**必须就"任务成功"达成一致**，否则会出现"浏览器里说完成了，离线验证却判定失败"的矛盾。

## 当前状态（2026-08-26 快照）

| Checker | Hub 后端 | JS 端 (`AxisWebInfra`) | Python 端 (`AxisDataCleaning`) |
|---------|:--------:|:----------------------:|:-----------------------------:|
| `joint_position` / `BoxJointPositionChecker` | ✅ | ✅ | ✅ |
| `gripper_open` / `GripperOpenChecker` | ✅ | ✅ | ✅ |
| `composite` / `CompositeChecker` | ✅ | ✅ | ✅ |
| `RelativePositionBoundsChecker` | ✅ | ⏳ PR | ✅ |
| `PositionDeltaChecker` | ✅ | ⏳ PR | ⏳ PR |
| `OrderedStepChecker` | ✅ | ⏳ PR | ⏳ PR |
| `BodyContactChecker` | ✅ | ⏳ PR | ⏳ PR |
| `BowlPositionChecker` | ❌ | ❌ | ✅ |
| `DrawerPositionChecker` | ❌ | ❌ | ✅ |
| `SampleRotationChecker` | ❌ | ❌ | ✅ |
| `SamplePositionDeltaChecker` | ❌ | ❌ | ✅ |
| `KeyPressSetChecker` | ❌ | ❌ | ✅ |

### 图例
- ✅ = 已实现
- ⏳ PR = 本系列 PR 正在添加
- ❌ = 未实现（可能是历史遗留 / 专用）

## 问题

1. **JS 端长期落后**：只有 3 个基础 checker，而 Hub 后端和 Python 端各有一堆。
2. **专用 checker 漂移**：`BowlPositionChecker` 等只在 Python 端存在，说明曾经某个任务离线验证需要但前端不需要（任务已 ended），但未来可能复用。
3. **无跨仓同步机制**：没有人改动一个 checker 时提醒其他仓。

## 建议

1. **合并本系列 PR** 后，JS 端将拥有全部 6 个 Hub 常用 checker。
2. **将本文件作为 `docs/checker-parity.md` 提交**，让三方状态透明。
3. **长期**：在 GitHub Actions 里加一个"checker 名称集合校验" job（对比 JS `index.js` 的 registry 和 Python `run_checker` 的 dispatch），防止 drift。见 `docs/checker-parity-ci.md`（后续 PR）。

## 新增的 3 个 checker 用途

### `BodyContactChecker`
判定两个 body 是否接触（或距离 ≤ 阈值）。用于"夹爪夹住物体"、"盖子盖上了"这类需要**接触/近距**语义的任务。

```jsonc
{ "type": "BodyContactChecker", "body_a": "...", "body_b": "...", "max_contact_distance": 0.02 }
```

### `PositionDeltaChecker`
判定 body 是否相对基线移动超过阈值。用于"旋转了 30°"、"移动了 2cm"这类需要**位移**语义的任务。

```jsonc
{ "type": "PositionDeltaChecker", "sample_body": "...", "axes": ["x","y"], "min_delta_x": 0.018, "capture_runtime_initial": true }
```

### `OrderedStepChecker`
按顺序执行多个子 checker，前一个通过才检查下一个。用于"先拿起来，再放进去"这类**多阶段**任务。

```jsonc
{ "type": "OrderedStepChecker", "steps": [ {...step1...}, {...step2...} ] }
```

---

*维护者：如合并，请将本文件放在 `AxisWebInfra/docs/checker-parity.md`，并在改动 checker 时更新表格。*
