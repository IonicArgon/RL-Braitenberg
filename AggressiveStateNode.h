#pragma once

#include "StateNode.h"

class AggressiveStateNode : public StateNode {
 public:
  /**
   * @brief Overrode entry procedure for Aggressive state.
   * Runs entry code to set up Aggressive state before entering
   * main procedure.
   */
  void enter(VehicleContext& ctx) override;

  /**
   * @brief Overrode main procedure for Aggressive state.
   * Executed every FSM tick and behaves based on Aggressive state
   * described in lecture.
   */
  void execute(VehicleContext& ctx) override;

  /**
   * @brief Overrode exit procedure for Aggressive state.
   * Runs any cleanup exiting the Aggressive state before any
   * state transitions occur.
   */
  void exit(VehicleContext& ctx) override;

  /**
   * @returns The state enum for the Aggressive state, `AGGRESSIVE`.
   */
  StateEnum get_enum(void) const override;

 private:
  const float m_max_speed = 1.0f;
};
