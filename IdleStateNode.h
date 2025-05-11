#pragma once

#include "StateNode.h"

class IdleStateNode : public StateNode {
 public:
  /**
   * @brief Overrode entry procedure for Idle state.
   * Runs entry code to set up Idle state before entering
   * main procedure.
   */
  void enter(VehicleContext& ctx) override;

  /**
   * @brief Overrode main procedure for Idle state.
   * Executed every FSM tick and behaves based on Idle state
   * described in lecture.
   */
  void execute(VehicleContext& ctx) override;

  /**
   * @brief Overrode exit procedure for Idle state.
   * Runs any cleanup exiting the Idle state before any
   * state transitions occur.
   */
  void exit(VehicleContext& ctx) override;

  /**
   * @returns The state enum for the Idle state, `IDLE`.
   */
  StateEnum get_enum(void) const override;
};
