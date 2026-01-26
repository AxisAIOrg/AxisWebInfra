import { NextRequest, NextResponse } from 'next/server';
import { promises as fs } from 'fs';
import path from 'path';

type Task = {
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
  steps: { order: number; description: string }[] | null;
  rarity: string | null;
  base_points: number | null;
  category: string | null;
};

type TasksData = {
  tasks: Task[];
};

export async function GET(
  request: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  try {
    const { id } = await params;
    const taskId = parseInt(id, 10);

    if (isNaN(taskId)) {
      return NextResponse.json(
        { error: 'Invalid task ID' },
        { status: 400 }
      );
    }

    // Read tasks from local JSON file
    const tasksFilePath = path.join(process.cwd(), 'public', 'data', 'tasks.json');
    const tasksFileContent = await fs.readFile(tasksFilePath, 'utf-8');
    const tasksData: TasksData = JSON.parse(tasksFileContent);

    // Find the task by ID
    const task = tasksData.tasks.find(t => t.id === taskId);

    if (!task) {
      return NextResponse.json(
        { error: 'Task not found' },
        { status: 404 }
      );
    }

    // If task has mjcf_xml path, read the XML content
    if (task.mjcf_xml && !task.mjcf_xml.startsWith('<')) {
      try {
        const xmlPath = path.join(process.cwd(), 'public', 'mujoco-assets', task.mjcf_xml);
        const xmlContent = await fs.readFile(xmlPath, 'utf-8');
        task.mjcf_xml = xmlContent;
      } catch (error) {
        console.error(`[API] Failed to read XML file for task ${taskId}:`, error);
        // Keep the original path if file read fails
      }
    }

    return NextResponse.json(task);
  } catch (error) {
    console.error('[API] Error fetching task:', error);
    return NextResponse.json(
      { error: 'Internal server error' },
      { status: 500 }
    );
  }
}
