/**
 * Local Task API - 本地任务数据接口
 * 用于开源精简版，从本地 JSON 文件读取任务数据
 */

export type TaskStep = {
  order: number;
  description: string;
};

export type Task = {
  id: number;
  name: string;
  description: string;
  difficulty_stars: number;
  expected_duration: number;
  success_rate: number;
  thumbnail: string;
  time_limit: number | null;
  mjcf_xml: string | null;
  checker_config: Record<string, any> | null;
  initial_state: Record<string, any> | null;
  steps: TaskStep[] | null;
  rarity: string | null;
  base_points: number | null;
  category: string | null;
};

type TasksData = {
  tasks: Task[];
};

let cachedTasks: Task[] | null = null;

/**
 * 获取所有任务列表
 */
export async function getTasks(): Promise<Task[]> {
  if (cachedTasks) {
    return cachedTasks;
  }

  try {
    const response = await fetch('/data/tasks.json');
    if (!response.ok) {
      throw new Error(`Failed to load tasks: ${response.status}`);
    }
    const data: TasksData = await response.json();
    cachedTasks = data.tasks;
    return data.tasks;
  } catch (error) {
    console.error('[local-task-api] Failed to load tasks:', error);
    return [];
  }
}

/**
 * 根据 ID 获取单个任务
 */
export async function getTask(id: number): Promise<Task | null> {
  const tasks = await getTasks();
  return tasks.find(t => t.id === id) || null;
}

/**
 * 下载轨迹数据为 JSON 文件
 */
export function downloadTrajectory(trajectory: any, filename?: string): void {
  const blob = new Blob([JSON.stringify(trajectory, null, 2)], { 
    type: 'application/json' 
  });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename || `trajectory_${Date.now()}.json`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

/**
 * 保存轨迹到本地存储（可选）
 */
export function saveTrajectoryToLocal(taskId: number, trajectory: any): void {
  try {
    const key = `trajectory_task_${taskId}_${Date.now()}`;
    localStorage.setItem(key, JSON.stringify(trajectory));
    console.log(`[local-task-api] Trajectory saved to localStorage: ${key}`);
  } catch (error) {
    console.error('[local-task-api] Failed to save trajectory to localStorage:', error);
  }
}

/**
 * 获取本地存储的轨迹列表
 */
export function getLocalTrajectories(taskId?: number): { key: string; data: any }[] {
  const trajectories: { key: string; data: any }[] = [];
  
  try {
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key && key.startsWith('trajectory_task_')) {
        if (taskId === undefined || key.includes(`trajectory_task_${taskId}_`)) {
          const data = localStorage.getItem(key);
          if (data) {
            trajectories.push({ key, data: JSON.parse(data) });
          }
        }
      }
    }
  } catch (error) {
    console.error('[local-task-api] Failed to load trajectories from localStorage:', error);
  }
  
  return trajectories;
}
