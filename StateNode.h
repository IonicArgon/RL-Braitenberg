#pragma once
#include "Globals.h"

// vehicle context defined outside later
class VehicleContext;

/**
 * @brief Abstract base class representing a state in the vehicle FSM.
 * @note Do not use this directly. You must create derived child classes.
 */
class StateNode {
 public:
  virtual ~StateNode() = default;

  /**
   * @brief Called once by VehicleContext when transitioning into this
   * state. Use for state-specific setup and initiation actions, etc.
   * Default implementation does nothing.
   * @param ctx Reference to main vehicle context object.
   */
  virtual void enter(VehicleContext& ctx) {}

  /**
   * @brief Called repeatedly by VehicleContext while in this state.
   * Must be implemented by derived state classes.
   * Do not perform blocking actions within this method.
   * @param ctx Reference to main vehicle context object.
   */
  virtual void execute(VehicleContext& ctx) = 0;

  /**
   * @brief Called once by VehicleContext when transitioning out
   * of this state. Use for state-specific cleanup, modifications of
   * probabilities before transitions, etc. Default implementation
   * does nothing.
   * @param ctx Reference to main vehicle context object.
   */
  virtual void exit(VehicleContext& ctx) {}

  /**
   * @brief Return the enum identifier corresponding to the state class.
   * Must be implemented by derived state classes.
   * @returns The StateEnum value for this state.
   */
  virtual StateEnum get_enum(void) const = 0;
};
