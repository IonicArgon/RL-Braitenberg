#pragma once

#include "StateNode.h"

class ExplorerStateNode : public StateNode {
 public:
  /**
   * @brief Overrode entry procedure for Explorer state.
   * Runs entry code to set up Explorer state before entering
   * main procedure.
   */
  void enter(VehicleContext& ctx) override;

  /**
   * @brief Overrode main procedure for Explorer state.
   * Executed every FSM tick and behaves based on Explorer state
   * described in lecture.
   */
  void execute(VehicleContext& ctx) override;

  /**
   * @brief Overrode exit procedure for Explorer state.
   * Runs any cleanup exiting the Explorer state before any
   * state transitions occur.
   */
  void exit(VehicleContext& ctx) override;

  /**
   * @returns The state enum for the Explorer state, `EXPLORER`.
   */
  StateEnum get_enum(void) const override;

 public:
  const float m_max_speed = 1.0f;
};
