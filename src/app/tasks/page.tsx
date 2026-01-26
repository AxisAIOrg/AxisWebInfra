"use client";

import { useState, useEffect } from "react";
import Link from "next/link";
import { getTasks, type Task } from "@/lib/local-task-api";

function DifficultyStars({ value }: { value: number }) {
  return (
    <div className="flex text-black">
      {Array.from({ length: 5 }).map((_, i) => (
        <svg
          key={i}
          className={`w-3.5 h-3.5 ${
            i < value ? "fill-current" : "fill-none"
          }`}
          viewBox="0 0 20 20"
        >
          <path d="M9.049 2.927c.3-.921 1.603-.921 1.902 0l1.07 3.292a1 1 0 00.95.69h3.462c.969 0 1.371 1.24.588 1.81l-2.8 2.034a1 1 0 00-.364 1.118l1.07 3.292c.3.921-.755 1.688-1.54 1.118l-2.8-2.034a1 1 0 00-1.175 0l-2.8 2.034c-.784.57-1.838-.197-1.539-1.118l1.07-3.292a1 1 0 00-.364-1.118L2.98 8.72c-.783-.57-.38-1.81.588-1.81h3.461a1 1 0 00.951-.69l1.07-3.292z" />
        </svg>
      ))}
    </div>
  );
}

function getDifficultyLabel(difficulty: number): string {
  const labels: Record<number, string> = {
    1: "Beginner",
    2: "Easy",
    3: "Medium",
    4: "Hard",
    5: "Expert",
  };
  return labels[difficulty] || "Beginner";
}

function TaskCard({ task }: { task: Task }) {
  const estimatedTime = `${Math.floor(task.expected_duration / 60)}~${Math.ceil((task.expected_duration + 60) / 60)} min`;
  const hasTimeLimit = task.time_limit != null && task.time_limit > 0;

  return (
    <div className="bg-white border-2 border-gray-200 rounded-xl p-6 hover:shadow-lg hover:border-blue-300 transition-all">
      {/* Header */}
      <div className="flex justify-between items-start mb-4">
        <h3 className="font-bold text-xl text-gray-900">
          {task.name}
        </h3>
        {hasTimeLimit && (
          <div className="bg-amber-50 text-amber-600 border border-amber-200 text-xs font-bold px-2 py-1 rounded-md flex items-center gap-1">
            <svg className="w-3 h-3" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth="2.5" d="M12 8v4l3 2m6-2a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
            {Math.floor(task.time_limit! / 60)}:{(task.time_limit! % 60).toString().padStart(2, "0")}
          </div>
        )}
      </div>

      {/* Description */}
      <p className="text-sm text-gray-600 mb-6 line-clamp-3 min-h-[60px]">
        {task.description}
      </p>

      {/* Info Grid */}
      <div className="space-y-3 mb-6">
        <div className="flex items-center text-sm text-gray-700">
          <span className="w-24 text-gray-500">Difficulty:</span>
          <div className="flex items-center gap-2">
            <DifficultyStars value={task.difficulty_stars} />
            <span className="text-xs text-gray-500">({getDifficultyLabel(task.difficulty_stars)})</span>
          </div>
        </div>
        <div className="flex items-center text-sm text-gray-700">
          <span className="w-24 text-gray-500">Est. Time:</span>
          <span>{estimatedTime}</span>
        </div>
        {task.base_points && (
          <div className="flex items-center text-sm text-gray-700">
            <span className="w-24 text-gray-500">Points:</span>
            <span className="flex items-center gap-1">
              <span>💎</span>
              <span className="font-semibold">{task.base_points}</span>
            </span>
          </div>
        )}
      </div>

      {/* Action Button */}
      <Link
        href={`/task-running?id=${task.id}`}
        className="block w-full py-3 bg-blue-500 hover:bg-blue-600 text-white font-semibold rounded-lg text-center transition-colors"
      >
        Start Task
      </Link>
    </div>
  );
}

export default function TasksPage() {
  const [tasks, setTasks] = useState<Task[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    async function loadTasks() {
      try {
        const loadedTasks = await getTasks();
        setTasks(loadedTasks);
      } catch (error) {
        console.error("[TasksPage] Failed to load tasks:", error);
      } finally {
        setLoading(false);
      }
    }

    loadTasks();
  }, []);

  return (
    <main className="max-w-[1200px] mx-auto px-6 py-12 bg-white min-h-screen">
      {/* Header */}
      <div className="text-center mb-12">
        <h1 className="text-4xl font-bold text-gray-900 mb-4">
          Robot Control Tasks
        </h1>
        <p className="text-lg text-gray-600 max-w-2xl mx-auto">
          Practice controlling a Franka Emika Panda robotic arm using keyboard controls. 
          Complete tasks to learn teleoperation skills.
        </p>
      </div>

      {/* Task Grid */}
      {loading ? (
        <div className="flex items-center justify-center py-20">
          <div className="text-gray-500">Loading tasks...</div>
        </div>
      ) : tasks.length > 0 ? (
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
          {tasks.map((task) => (
            <TaskCard key={task.id} task={task} />
          ))}
        </div>
      ) : (
        <div className="text-center py-20">
          <p className="text-gray-500">No tasks available.</p>
          <p className="text-sm text-gray-400 mt-2">
            Check that /public/data/tasks.json exists and contains task data.
          </p>
        </div>
      )}

      {/* Instructions */}
      <div className="mt-16 p-8 bg-gray-50 rounded-xl">
        <h2 className="text-2xl font-bold text-gray-900 mb-6">
          How to Control the Robot
        </h2>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
          <div>
            <h3 className="font-semibold text-gray-800 mb-2">Move Gripper</h3>
            <ul className="text-sm text-gray-600 space-y-1">
              <li><kbd className="px-2 py-1 bg-gray-200 rounded">↑↓←→</kbd> Translate XY</li>
              <li><kbd className="px-2 py-1 bg-gray-200 rounded">E</kbd> / <kbd className="px-2 py-1 bg-gray-200 rounded">D</kbd> Up/Down</li>
            </ul>
          </div>
          <div>
            <h3 className="font-semibold text-gray-800 mb-2">Rotate Gripper</h3>
            <ul className="text-sm text-gray-600 space-y-1">
              <li><kbd className="px-2 py-1 bg-gray-200 rounded">Q</kbd> / <kbd className="px-2 py-1 bg-gray-200 rounded">W</kbd> Roll</li>
              <li><kbd className="px-2 py-1 bg-gray-200 rounded">A</kbd> / <kbd className="px-2 py-1 bg-gray-200 rounded">S</kbd> Pitch</li>
              <li><kbd className="px-2 py-1 bg-gray-200 rounded">Z</kbd> / <kbd className="px-2 py-1 bg-gray-200 rounded">X</kbd> Yaw</li>
            </ul>
          </div>
          <div>
            <h3 className="font-semibold text-gray-800 mb-2">Actions</h3>
            <ul className="text-sm text-gray-600 space-y-1">
              <li><kbd className="px-2 py-1 bg-gray-200 rounded">Space</kbd> Toggle gripper</li>
              <li><kbd className="px-2 py-1 bg-gray-200 rounded">R</kbd> Reset episode</li>
            </ul>
          </div>
          <div>
            <h3 className="font-semibold text-gray-800 mb-2">Camera</h3>
            <ul className="text-sm text-gray-600 space-y-1">
              <li>Left drag: Orbit</li>
              <li>Right drag: Pan</li>
              <li>Scroll: Zoom</li>
            </ul>
          </div>
        </div>
      </div>
    </main>
  );
}
