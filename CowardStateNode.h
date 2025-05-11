#pragma once

#include "StateNode.h"

class CowardStateNode : public StateNode {
 public:
  /**
   * @brief Overrode entry procedure for Coward state.
   * Runs entry code to set up Coward state before entering
   * main procedure.
   */
  void enter(VehicleContext& ctx) override;

  /**
   * @brief Overrode main procedure for Coward state.
   * Executed every FSM tick and behaves based on Coward state
   * described in lecture.
   */
  void execute(VehicleContext& ctx) override;

  /**
   * @brief Overrode exit procedure for Coward state.
   * Runs any cleanup exiting the Coward state before any
   * state transitions occur.
   */
  void exit(VehicleContext& ctx) override;

  /**
   * @returns The state enum for the Coward state, `COWARD`.
   */
  StateEnum get_enum(void) const override;

 public:
  const float m_max_speed = 1.0f;
};
