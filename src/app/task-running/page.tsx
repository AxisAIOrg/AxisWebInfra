"use client";

import { Suspense, useState, useEffect, useRef, useMemo } from "react";
import { useRouter } from "next/navigation";
import { useSearchParams } from "next/navigation";

import { MuJoCoDemo } from "@/components/mujoco-framework-next/main.js";
import { getTask, downloadTrajectory, type Task } from "@/lib/local-task-api";
import { getTaskMetadata } from "@/lib/task-metadata";

type MissionStep = {
  id: number;
  label: string;
  completed: boolean;
};

type DemoLoadingProgress = {
  phase: string;
  message?: string;
  loaded?: number;
  total?: number;
  percent?: number;
  currentFile?: string;
};

function formatTime(seconds: number): string {
  const mins = Math.floor(seconds / 60);
  const secs = Math.floor(seconds % 60);
  return `${String(mins).padStart(2, "0")}:${String(secs).padStart(2, "0")}`;
}

type ControlsPanelItem = {
  keys: string[];
  label: string;
  description?: string;
};

type ControlsPanelSection = {
  title: string;
  items: ControlsPanelItem[];
};

function humanizeKey(codeOrLabel: string): string {
  if (!codeOrLabel) return "";
  const upper = String(codeOrLabel).toUpperCase();
  if (upper === "SPACE") return "SPACE";
  if (upper === "ESC" || upper === "ESCAPE") return "ESC";
  if (codeOrLabel === "ArrowUp") return "↑";
  if (codeOrLabel === "ArrowDown") return "↓";
  if (codeOrLabel === "ArrowLeft") return "←";
  if (codeOrLabel === "ArrowRight") return "→";
  if (codeOrLabel === "Space") return "SPACE";
  if (/^Key[A-Z]$/.test(codeOrLabel)) return codeOrLabel.replace("Key", "");
  if (/^Digit[0-9]$/.test(codeOrLabel)) return codeOrLabel.replace("Digit", "");
  return codeOrLabel;
}

function getDefaultControlsSections(): ControlsPanelSection[] {
  return [
    {
      title: "Move gripper",
      items: [
        { keys: ["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"], label: "Translate (XY)" },
        { keys: ["KeyE", "KeyD"], label: "Up / Down" },
      ],
    },
    {
      title: "Rotate gripper",
      items: [
        { keys: ["KeyQ", "KeyW"], label: "Roll (+ / -)" },
        { keys: ["KeyA", "KeyS"], label: "Pitch (+ / -)" },
        { keys: ["KeyZ", "KeyX"], label: "Yaw (+ / -)" },
      ],
    },
    {
      title: "Actions",
      items: [
        { keys: ["Space"], label: "Toggle gripper" },
        { keys: ["KeyR"], label: "Reset episode" },
      ],
    },
    {
      title: "Camera",
      items: [
        { keys: ["Mouse"], label: "Left drag: orbit · Right drag: pan · Scroll: zoom" },
      ],
    },
  ];
}

function TaskRunningPageContent() {
  const searchParams = useSearchParams();
  const router = useRouter();
  const idParam = searchParams?.get("id");
  const taskId = Number(idParam);
  
  const [task, setTask] = useState<Task | null>(null);
  const [status, setStatus] = useState({
    time: 0,
    trajectoryCount: 0,
    isSuccess: false,
    currentTrajectory: {
      recording: false,
      samples: 0,
      durationMs: 0,
      durationSec: "0.0",
    },
    completedTrajectories: 0,
    trajectorySteps: [] as any[],
  });
  const [isCompleted, setIsCompleted] = useState(false);
  const [finalStatus, setFinalStatus] = useState<any>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const [showSuccessDialog, setShowSuccessDialog] = useState<boolean>(false);
  const [fps, setFps] = useState(60);
  const [missionSteps, setMissionSteps] = useState<MissionStep[]>([]);
  const [demoLoading, setDemoLoading] = useState<boolean>(false);
  const [demoLoadingProgress, setDemoLoadingProgress] = useState<DemoLoadingProgress | null>(null);
  const [lastTrajectory, setLastTrajectory] = useState<any>(null);
  
  const containerRef = useRef<HTMLDivElement | null>(null);
  const demoRef = useRef<MuJoCoDemo | null>(null);
  const taskMetadata = useMemo(() => getTaskMetadata(taskId), [taskId]);
  
  const displayStatus = isCompleted && finalStatus ? finalStatus : status;
  const goalAchieved = displayStatus.isSuccess;
  const simulationTime = displayStatus.time;
  const timerSeconds = Math.floor(simulationTime / 1000);
  const controlsPanelSections = getDefaultControlsSections();

  // Fetch task data
  useEffect(() => {
    if (!idParam || Number.isNaN(taskId) || taskId <= 0) {
      setError("Invalid task ID");
      setLoading(false);
      return;
    }

    getTask(taskId)
      .then((taskData) => {
        if (!taskData) {
          setError("Task not found");
          return;
        }
        
        setTask(taskData);
        
        // Initialize mission steps from task steps
        if (taskData.steps && taskData.steps.length > 0) {
          const sortedSteps = [...taskData.steps].sort((a, b) => a.order - b.order);
          setMissionSteps(
            sortedSteps.map((step) => ({
              id: step.order,
              label: step.description,
              completed: false,
            }))
          );
        } else {
          setMissionSteps([]);
        }
        
        setLoading(false);
      })
      .catch((err) => {
        setError(err instanceof Error ? err.message : "Failed to load task");
        setLoading(false);
      });
  }, [idParam, taskId]);

  // Initialize MuJoCo demo
  useEffect(() => {
    if (!task || isCompleted) return;
    if (!containerRef.current) return;

    let cancelled = false;
    const parentElement = containerRef.current;
    let demo: MuJoCoDemo | null = null;

    demo = new MuJoCoDemo({
      parentElement,
      checkerConfig: task.checker_config || null,
      initialState: task.initial_state || null,
      domainRandomization: taskMetadata?.domain_randomization || null,
      onLoadingProgress: (p: DemoLoadingProgress) => {
        if (cancelled) return;
        setDemoLoadingProgress(p);
        if (p?.phase === "ready") {
          setDemoLoading(false);
        } else if (p?.phase === "error") {
          setDemoLoading(false);
        } else {
          setDemoLoading(true);
        }
      },
    });

    demo.onStatusUpdate = (statusData: any) => {
      if (!cancelled && !isCompleted) {
        setStatus(statusData);
        if (statusData.fps) {
          setFps(Math.round(statusData.fps));
        }
      }
    };

    demo.onTrajectoryExport = (trajectory: any[], metadata: any) => {
      if (cancelled) return;
      
      setStatus((currentStatus) => {
        setIsCompleted(true);
        setFinalStatus(currentStatus);
        return currentStatus;
      });

      // Store trajectory for download
      setLastTrajectory({
        trajectory,
        metadata,
        exportedAt: new Date().toISOString(),
      });
      
      setShowSuccessDialog(true);
    };

    demoRef.current = demo;

    (async () => {
      try {
        setDemoLoading(true);
        setDemoLoadingProgress({ phase: "init", message: "Initializing simulation…", percent: 0 });
        await demo!.init();
      } catch (error) {
        if (!cancelled) {
          console.error("[task-running] Failed to init MuJoCoDemo:", error);
          setError("Failed to initialize simulation");
          setDemoLoading(false);
        }
      }
    })();

    return () => {
      cancelled = true;
      if (!demo) return;

      if (demo.stopStatusBroadcasting) {
        demo.stopStatusBroadcasting();
      }

      if ((demo as any).resizeObserver) {
        (demo as any).resizeObserver.disconnect();
      }

      try {
        if ((demo as any).renderer?.setAnimationLoop) {
          (demo as any).renderer.setAnimationLoop(null);
        }
        if ((demo as any).container?.parentElement) {
          (demo as any).container.parentElement.removeChild((demo as any).container);
        }
      } catch (e) {
        console.warn("[task-running] Cleanup warning:", e);
      }

      demoRef.current = null;
    };
  }, [task, isCompleted, taskMetadata]);

  // Update mission steps on success
  useEffect(() => {
    if (goalAchieved) {
      setMissionSteps((steps) =>
        steps.map((step) => ({ ...step, completed: true }))
      );
    }
  }, [goalAchieved]);

  // Handle timeout
  useEffect(() => {
    if (!task?.time_limit || isCompleted || goalAchieved) return;

    if (timerSeconds >= task.time_limit) {
      console.log("[task-running] Time limit reached!");
      setIsCompleted(true);
      setFinalStatus(status);
    }
  }, [timerSeconds, task?.time_limit, isCompleted, goalAchieved, status]);

  const handleDownloadTrajectory = () => {
    if (lastTrajectory) {
      downloadTrajectory(lastTrajectory, `trajectory_task_${taskId}_${Date.now()}.json`);
    }
  };

  const handleReset = () => {
    setIsCompleted(false);
    setFinalStatus(null);
    setShowSuccessDialog(false);
    setLastTrajectory(null);
    setMissionSteps((steps) => steps.map((s) => ({ ...s, completed: false })));
    
    if (demoRef.current) {
      demoRef.current.reset();
    }
  };

  if (loading) {
    return (
      <div className="relative w-full h-screen bg-[#020518] text-[#00FF47] font-mono flex items-center justify-center">
        <div className="text-xl opacity-50">Loading...</div>
      </div>
    );
  }

  if (error || !task) {
    return (
      <div className="relative w-full h-screen bg-[#020518] text-[#00FF47] font-mono flex items-center justify-center">
        <div className="text-center">
          <div className="text-xl text-red-500 mb-4">{error || "Task not found"}</div>
          <button
            onClick={() => router.push("/tasks")}
            className="px-6 py-2 bg-[#00FF47] text-black font-bold rounded-lg"
          >
            Back to Tasks
          </button>
        </div>
      </div>
    );
  }

  const progressPercent =
    typeof demoLoadingProgress?.percent === "number"
      ? Math.max(0, Math.min(100, demoLoadingProgress.percent))
      : typeof demoLoadingProgress?.loaded === "number" &&
          typeof demoLoadingProgress?.total === "number" &&
          demoLoadingProgress.total > 0
        ? Math.max(0, Math.min(100, Math.round((demoLoadingProgress.loaded / demoLoadingProgress.total) * 100)))
        : null;

  return (
    <>
      {/* Loading Overlay */}
      {demoLoading && (
        <div className="fixed inset-0 z-[60] flex items-center justify-center p-4">
          <div className="absolute inset-0 bg-black/70 backdrop-blur-sm" />
          <div className="relative z-10 w-full max-w-xl rounded-2xl border border-[#00FF47] bg-[#020518] p-8">
            <div className="mb-2 text-sm uppercase tracking-wider text-[#00FF47]/70">
              Loading MuJoCo Demo
            </div>
            <div className="mb-4 text-lg font-bold text-[#00FF47]">
              {demoLoadingProgress?.message || "Loading…"}
            </div>

            {demoLoadingProgress?.currentFile && (
              <div className="mb-4 font-mono text-xs text-[#00FF47]/70 break-all">
                {demoLoadingProgress.currentFile}
              </div>
            )}

            <div className="w-full h-3 border border-[#00FF47] bg-black/40 overflow-hidden">
              {progressPercent === null ? (
                <div className="h-full w-1/3 bg-[#00FF47] animate-pulse" />
              ) : (
                <div
                  className="h-full bg-[#00FF47] transition-[width] duration-200"
                  style={{ width: `${progressPercent}%` }}
                />
              )}
            </div>

            <div className="mt-3 flex items-center justify-between text-xs text-[#00FF47]/70">
              <div className="font-mono">
                {typeof demoLoadingProgress?.loaded === "number" &&
                typeof demoLoadingProgress?.total === "number"
                  ? `${demoLoadingProgress.loaded}/${demoLoadingProgress.total}`
                  : demoLoadingProgress?.phase || "loading"}
              </div>
              <div className="font-mono">
                {progressPercent === null ? "…" : `${progressPercent}%`}
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Success Dialog */}
      {showSuccessDialog && (
        <div
          className="fixed inset-0 z-50 flex items-center justify-center p-4"
          onClick={() => setShowSuccessDialog(false)}
        >
          <div className="absolute inset-0 bg-black/50 backdrop-blur-sm" />
          <div
            className="relative z-10 w-full max-w-md rounded-2xl border border-[#00FF47] bg-[#020518] p-8"
            onClick={(e) => e.stopPropagation()}
          >
            <h2 className="mb-4 text-center text-2xl font-bold text-[#00FF47]">
              🎉 Task Completed!
            </h2>
            <p className="mb-2 text-center text-[#00FF47]/80">
              Time: {formatTime(timerSeconds)}
            </p>
            <p className="mb-6 text-center text-[#00FF47]/60 text-sm">
              Trajectory recorded with {lastTrajectory?.trajectory?.length || 0} samples
            </p>
            
            <div className="space-y-3">
              <button
                onClick={handleDownloadTrajectory}
                className="w-full py-3 bg-[#00FF47] text-black font-bold rounded-lg hover:bg-[#00FF47]/80 transition-colors"
              >
                Download Trajectory
              </button>
              <button
                onClick={handleReset}
                className="w-full py-3 bg-transparent border border-[#00FF47] text-[#00FF47] font-bold rounded-lg hover:bg-[#00FF47]/10 transition-colors"
              >
                Try Again
              </button>
              <button
                onClick={() => router.push("/tasks")}
                className="w-full py-3 bg-transparent text-[#00FF47]/60 font-medium rounded-lg hover:text-[#00FF47] transition-colors"
              >
                Back to Tasks
              </button>
            </div>
          </div>
        </div>
      )}

      <div className="relative w-full h-screen bg-[#020518] text-[#00FF47] font-mono overflow-hidden select-none">
        {/* MuJoCo Canvas - Full Background */}
        <div
          ref={containerRef}
          className="absolute inset-0 w-full h-full z-0"
          style={{ width: "100%", height: "100%", position: "absolute", top: 0, left: 0, right: 0, bottom: 0 }}
        />
        <div className="absolute inset-0 bg-[linear-gradient(rgba(0,255,71,0.03)_1px,transparent_1px),linear-gradient(90deg,rgba(0,255,71,0.03)_1px,transparent_1px)] bg-[size:50px_50px] pointer-events-none z-0"></div>

        {/* Header */}
        <header className="absolute top-0 left-0 w-full h-14 flex items-center justify-between px-6 border-b border-[#00FF47]/30 z-50 bg-[#020518]/80 backdrop-blur-sm">
          <div className="flex flex-col gap-1 min-w-0">
            <div className="text-xl font-bold tracking-wide truncate">
              Task {task.id}: {task.name}
            </div>
          </div>
          <div className="absolute left-1/2 -translate-x-1/2 flex flex-col items-center gap-1">
            <div className="text-3xl font-bold tracking-widest drop-shadow-[0_0_5px_rgba(0,255,71,0.5)]">
              {formatTime(timerSeconds)}
            </div>
            {task.time_limit != null && (
              <div className="text-xs text-[#00FF47]/70 uppercase tracking-wider">
                Limit: {formatTime(task.time_limit)}
                {timerSeconds >= task.time_limit && (
                  <span className="ml-2 text-red-400 animate-pulse">TIME UP</span>
                )}
              </div>
            )}
          </div>
          <div className="flex items-center gap-6 text-sm">
            <div className="opacity-80">FPS: {fps}</div>
            <button
              onClick={() => router.push("/tasks")}
              className="text-[#00FF47] hover:text-white transition-colors"
            >
              <svg
                className="w-8 h-8"
                fill="none"
                stroke="currentColor"
                viewBox="0 0 24 24"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth="3"
                  d="M6 18L18 6M6 6l12 12"
                />
              </svg>
            </button>
          </div>
        </header>

        {/* Controls Panel - Top Left */}
        <div className="absolute top-20 left-6 w-[400px] border border-[#00FF47] bg-black/80 p-4 rounded-sm z-40">
          <div className="flex justify-between items-center mb-4 border-b border-[#00FF47]/30 pb-1">
            <span className="text-xs font-bold uppercase tracking-wider">Controls</span>
            <div className="w-10 h-0.5 bg-[#00FF47]"></div>
          </div>
          <div className="space-y-3">
            {controlsPanelSections.map((section) => (
              <div key={section.title} className="space-y-2">
                <div className="text-[10px] uppercase tracking-wider text-[#00FF47]/70">
                  {section.title}
                </div>
                <div className="space-y-2">
                  {section.items.map((item) => (
                    <div
                      key={`${section.title}:${item.label}`}
                      className="flex items-center justify-between gap-3 border border-[#00FF47]/30 px-2 py-1"
                    >
                      <div className="flex flex-wrap gap-1.5">
                        {item.keys.map((k) => (
                          <span
                            key={k}
                            className="inline-flex min-w-[28px] items-center justify-center border border-[#00FF47] px-1.5 py-0.5 text-[10px] font-bold"
                          >
                            {humanizeKey(k)}
                          </span>
                        ))}
                      </div>
                      <div className="text-xs text-right">
                        <div className="font-bold">{item.label}</div>
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Sidebar - Top Right */}
        <div className="absolute top-20 right-6 w-72 flex flex-col gap-4 z-40">
          {/* Mission Steps */}
          <div className="border border-[#00FF47] bg-black/80 p-4">
            <h3 className="text-xs uppercase opacity-70 border-b border-[#00FF47]/30 pb-2 mb-3">
              Mission Steps
            </h3>
            <ul className="space-y-3 text-sm">
              {missionSteps.length > 0 ? (
                missionSteps.map((step) => (
                  <li
                    key={step.id}
                    className={`flex items-start gap-2 ${
                      step.completed ? "opacity-100" : "opacity-50"
                    }`}
                  >
                    {step.completed ? (
                      <span className="text-[#00FF47]">✓</span>
                    ) : (
                      <span>{step.id}.</span>
                    )}
                    <span className={step.completed ? "line-through opacity-50" : ""}>
                      {step.label}
                    </span>
                  </li>
                ))
              ) : (
                <li className="text-xs opacity-50 italic">No steps defined</li>
              )}
            </ul>
          </div>

          {/* Status */}
          <div className="border border-[#00FF47] bg-black/80 p-4">
            <h3 className="text-xs uppercase opacity-70 border-b border-[#00FF47]/30 pb-2 mb-2">
              Status
            </h3>
            <div className="space-y-2 text-sm">
              <div className="flex justify-between">
                <span className="opacity-70">Recording:</span>
                <span>{displayStatus.currentTrajectory.recording ? "Yes" : "No"}</span>
              </div>
              <div className="flex justify-between">
                <span className="opacity-70">Samples:</span>
                <span>{displayStatus.currentTrajectory.samples}</span>
              </div>
              <div className="flex justify-between">
                <span className="opacity-70">Goal:</span>
                <span className={goalAchieved ? "text-[#00FF47]" : "text-red-400"}>
                  {goalAchieved ? "✓ Achieved" : "In Progress"}
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}

export default function TaskRunningPage() {
  return (
    <Suspense fallback={<div className="w-full h-screen bg-[#020518] flex items-center justify-center text-[#00FF47]">Loading...</div>}>
      <TaskRunningPageContent />
    </Suspense>
  );
}
