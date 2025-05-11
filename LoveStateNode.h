#pragma once

#include "StateNode.h"

class LoveStateNode : public StateNode {
 public:
  /**
   * @brief Overrode entry procedure for Love state.
   * Runs entry code to set up Love state before entering
   * main procedure.
   */
  void enter(VehicleContext& ctx) override;

  /**
   * @brief Overrode main procedure for Love state.
   * Executed every FSM tick and behaves based on Love state
   * described in lecture.
   */
  void execute(VehicleContext& ctx) override;

  /**
   * @brief Overrode exit procedure for Love state.
   * Runs any cleanup exiting the Love state before any
   * state transitions occur.
   */
  void exit(VehicleContext& ctx) override;

  /**
   * @returns The state enum for the Love state, `LOVE`.
   */
  StateEnum get_enum(void) const override;

 public:
  const float m_max_speed = 1.0f;
};
