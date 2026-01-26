/**
 * Task Metadata - Domain Randomization Configuration
 * 
 * 用于配置任务的域随机化参数，如物体位置偏移、旋转偏移等。
 * 当前开源版本只包含一个示例任务，此文件可用于扩展更多任务。
 */

export type DomainRandomizationConfig = {
  objects?: Record<string, any>;
  robots?: Record<string, any>;
};

export type TaskMetadata = {
  domain_randomization?: DomainRandomizationConfig;
};

/**
 * Task metadata registry
 * Add domain randomization config for each task here
 */
const TASK_METADATA: Record<number, TaskMetadata> = {
  // Task 1: Close the Box
  // 当前不需要域随机化，保持默认状态
};

export function getTaskMetadata(taskId: number | string | null | undefined): TaskMetadata | null {
  if (taskId == null) return null;
  const idNumber = typeof taskId === "string" ? Number(taskId) : taskId;
  if (!Number.isFinite(idNumber)) return null;
  return TASK_METADATA[idNumber] ?? null;
}
