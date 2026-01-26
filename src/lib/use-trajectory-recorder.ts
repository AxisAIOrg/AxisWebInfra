import { useRef, useCallback } from 'react';
import type { TrajectoryStep, TrajectoryState, TrajectoryAction } from './trajectory-types';

interface UseTrajectoryRecorderOptions {
  samplingRate?: number; // Record every N steps (default: 10)
  maxSteps?: number; // Maximum steps to keep in memory (default: 10000)
}

export function useTrajectoryRecorder(options: UseTrajectoryRecorderOptions = {}) {
  const { samplingRate = 10, maxSteps = 10000 } = options;
  
  const trajectoryRef = useRef<TrajectoryStep[]>([]);
  const stepCountRef = useRef<number>(0);
  const keysPressedRef = useRef<Set<string>>(new Set());
  const lastRecordedTimeRef = useRef<number>(0);

  const recordStep = useCallback((
    simulationTime: number,
    state: TrajectoryState,
    additionalAction?: Partial<TrajectoryAction>
  ) => {
    stepCountRef.current++;
    
    // Only record every N steps
    if (stepCountRef.current % samplingRate !== 0) {
      return;
    }
    
    // Prevent memory overflow
    if (trajectoryRef.current.length >= maxSteps) {
      // Remove oldest 10% of steps
      const removeCount = Math.floor(maxSteps * 0.1);
      trajectoryRef.current.splice(0, removeCount);
    }
    
    const action: TrajectoryAction = {
      keyboard: Array.from(keysPressedRef.current),
      ...additionalAction,
    };
    
    const step: TrajectoryStep = {
      timestamp: Date.now(),
      simulation_time: simulationTime,
      state,
      action,
    };
    
    trajectoryRef.current.push(step);
  }, [samplingRate, maxSteps]);

  const updateKeysPressed = useCallback((keys: Set<string>) => {
    keysPressedRef.current = keys;
  }, []);

  const getTrajectory = useCallback((): TrajectoryStep[] => {
    return [...trajectoryRef.current];
  }, []);

  const clearTrajectory = useCallback(() => {
    trajectoryRef.current = [];
    stepCountRef.current = 0;
    lastRecordedTimeRef.current = 0;
  }, []);

  const getStepCount = useCallback((): number => {
    return stepCountRef.current;
  }, []);

  return {
    recordStep,
    updateKeysPressed,
    getTrajectory,
    clearTrajectory,
    getStepCount,
  };
}

