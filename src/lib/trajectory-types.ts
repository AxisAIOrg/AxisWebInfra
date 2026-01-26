/**
 * Types for trajectory recording
 */

export interface TrajectoryState {
  robot_joints: Record<string, number>;
  robot_velocities: Record<string, number>;
  object_positions: Record<string, [number, number, number]>;
  object_velocities: Record<string, [number, number, number]>;
}

export interface TrajectoryAction {
  keyboard?: string[];
  mouse?: {
    type: string;
    x: number;
    y: number;
    button?: number;
  };
  controls?: Record<string, number>;
}

export interface TrajectoryStep {
  timestamp: number; // Unix timestamp in milliseconds
  simulation_time: number; // Simulation time in seconds
  state: TrajectoryState;
  action: TrajectoryAction;
}

export interface TrajectoryMetadata {
  total_steps: number;
  simulation_time: number;
  goal_achieved: boolean;
  frame_rate?: number;
  sampling_rate: number; // Record every N steps
  task_id: number;
  user_id: string;
}

export interface TrajectoryData {
  user_id: string;
  trajectory: TrajectoryStep[];
  metadata: TrajectoryMetadata;
}

