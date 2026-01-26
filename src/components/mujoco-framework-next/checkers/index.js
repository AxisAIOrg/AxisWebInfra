/**
 * Checker Registry
 * Central registry for all available checker types.
 * Main.js should not import individual checkers directly.
 */

import { GripperOpenChecker } from './GripperOpenChecker.js';
import { BoxJointPositionChecker } from './BoxJointPositionChecker.js';
import { CompositeChecker } from './CompositeChecker.js';

/**
 * Registry mapping checker type names to their classes
 */
export const CHECKER_REGISTRY = {
  // Full names
  GripperOpenChecker,
  BoxJointPositionChecker,
  CompositeChecker,
  // Aliases for task config
  joint_position: BoxJointPositionChecker,
  composite: CompositeChecker,
  gripper_open: GripperOpenChecker,
};

/**
 * Get checker class by type name
 */
export function getCheckerClass(typeName) {
  const CheckerClass = CHECKER_REGISTRY[typeName];
  if (!CheckerClass) {
    throw new Error(`Unknown checker type: ${typeName}. Available: ${Object.keys(CHECKER_REGISTRY).join(', ')}`);
  }
  return CheckerClass;
}
