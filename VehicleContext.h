#pragma once

#include "AggressiveStateNode.h"
#include "CommsContext.h"
#include "CowardStateNode.h"
#include "ExplorerStateNode.h"
#include "Globals.h"
#include "IdleStateNode.h"
#include "LoveStateNode.h"
#include "StateNode.h"

/**
 * @brief Main vehicle context for the Braitenberg vehicle.
 */
class VehicleContext {
 public:
  /**
   * @brief Constructor for the VehicleContext class.
   * @param ldr_l Pin for reading the left LDR.
   * @param ldr_r Pin for reading the right LDR.
   * @param ldr_l_gnd Ground pin for left LDR.
   * @param ldr_r_gnd Ground pin for right LDR.
   * @param mtr_l_in1 IN1 pin for the H-bridge driver, left side motor.
   * @param mtr_l_in2 IN2 pin for the H-bridge driver, left side motor.
   * @param mtr_r_in3 IN3 pin for the H-bridge driver, right side motor.
   * @param mtr_r_in4 IN4 pin for the H-bridge driver, right side motor.
   * @param mtr_l_pwm PWM pin for the H-bridge driver, left side motor.
   * @param mtr_r_pwm PWM pin for the H-bridge driver, right side motor.
   * @param led_g Pin for the onboard green LED.
   * @param led_r Pin for the onboard red LED.
   * @param learning_rate The step size for changing probabilities in the state
   * table.
   * @param ci_change_rate Unused.
   *
   */
  VehicleContext(PinName ldr_l, PinName ldr_r, PinName ldr_l_gnd,
                 PinName ldr_r_gnd, PinName mtr_l_in1, PinName mtr_l_in2,
                 PinName mtr_r_in3, PinName mtr_r_in4, PinName mtr_l_pwm,
                 PinName mtr_r_pwm, PinName led_g, PinName led_r,
                 float learning_rate = 0.1f, float ci_change_rate = 0.05f);

  /**
   * @brief Is called every FSM "tick". Calls StateNode::execute.
   */
  void run_fsm_cycle(void);

  /**
   * @brief Called by StateNode::execute to indicate what state to
   * transition into next. Handles learning updates, general cleanup, etc.
   * Calls StateNode::exit before performing transition.
   */
  void transition_to(StateEnum next_state);

  /**
   * @returns LightLevels struct containing left and right values of
   * photoresistors. Noramlized between 0.0f - 1.0f.
   */
  LightLevels get_curr_light_lvls(void) const;

  /**
   * @returns Elapsed time in current state as `Kernel::Clock::duration`.
   */
  Kernel::Clock::duration get_elapsed_time_in_state(void) const;

  /**
   * @returns The minimum duration for a given state as
   * `Kernel::Clock::duration`.
   */
  Kernel::Clock::duration get_min_duration(StateEnum state) const;

  /**
   * @brief Sets the direction and "speed" (PWM duty cycle) of the left and
   * right wheels. Direction parameters (`dir_x`) use the following characters:
   * `F` (forwards), `R` (reverse), `S` (stop).
   * @param dir_l Direction of the left wheel.
   * @param dir_r Direction of the right wheel.
   * @param pwm_l "Speed" of the left wheel as a PWM duty cycle (0.0f - 1.0f).
   * @param pwm_r "Speed" of the right wheel as a PWM duty cycle (0.0f - 1.0f).
   */
  void set_motor_speeds(Direction dir_l, Direction dir_r, float pwm_l,
                        float pwm_r);

  /**
   * @brief Updates the probability table using built-in reward mechanisms and
   * internal states. Reward mechanism based on minimizing light levels.
   */
  void update_probability_table(float reward);

  /**
   * @returns The next state based on current probabilities, optionally
   * influenced by communication.
   */
  StateEnum sample_next_state(void);

  /**
   * The CommsContext object for communication using the RF transceiver.
   */
  CommsContext m_comms_ctx;

 private:
  // for photoresistors
  AnalogIn m_ldr_l;
  AnalogIn m_ldr_r;
  DigitalOut m_ldr_l_gnd;
  DigitalOut m_ldr_r_gnd;

  // for motor control
  DigitalOut m_mtr_l_in1;
  DigitalOut m_mtr_l_in2;
  PwmOut m_mtr_l_pwm;
  DigitalOut m_mtr_r_in3;
  DigitalOut m_mtr_r_in4;
  PwmOut m_mtr_r_pwm;

  // state node instances
  IdleStateNode m_state_idle;
  AggressiveStateNode m_state_aggressive;
  CowardStateNode m_state_coward;
  ExplorerStateNode m_state_explorer;
  LoveStateNode m_state_love;
  StateNode* m_state_node_instances[NUM_STATES];

  // for internal FSM state and other values
  StateNode* m_curr_state_ptr;
  StateEnum m_curr_state;
  StateEnum m_prev_state;
  Kernel::Clock::time_point m_time_state_entry;
  LightLevels m_light_lvl_entry;
  LightLevels m_light_lvl_curr;
  LightLevels m_light_lvl_min;
  LightLevels m_light_lvl_max;

  // for learning and other things
  float m_probability_table[NUM_STATES][NUM_STATES];
  float m_comms_influence;
  const float m_learning_rate;
  const float m_ci_change_rate;

  // for miscellaneous configuration
  const Kernel::Clock::duration m_min_state_duration[NUM_STATES] = {
      2500ms,  // IDLE
      5000ms,  // TOWARDS_LIGHT
      5000ms,  // AWAY_LIGHT
      5000ms,  // TOWARDS_DARK
      5000ms,  // AWAY_DARK
  };
  DigitalOut m_g_led;
  DigitalOut m_r_led;

  /**
   * @brief Initializes the FSM, state tables, and prepares vehicle context for
   * running.
   */
  void initialize_fsm(void);

  /**
   * @param state The state requested.
   * @return A pointer to the `StateNode` object containing the requested state.
   */
  StateNode* get_state_node(StateEnum state) const;

  /**
   * @brief Reads values from the LDRs and writes them to `m_light_lvl_curr`.
   * Continually normalizes the values. Values are guaranteed to always be
   * between 0.0 - 1.0 (inclusive).
   */
  void read_sensors(void);

  /**
   * @brief Uses the previous and current light levels to compute a reward for
   * probability updates.
   * @param before The `LightLevels` object containing the light levels before
   * the state transition.
   * @param after The `LightLevels` object containing the light levels after
   * exiting the previous state.
   * @returns A floating point value representing the reward.
   */
  float calculate_reward(LightLevels before, LightLevels after);

  /**
   * @brief Internal function to normalize the probabilities such that:
   * - They always sum to 1.0.
   * - They are always between 0.01 - 1.0 (inclusive).
   * @param state The state to normalize their probability table for.
   */
  void normalize_probabilities(StateEnum state);

  /**
   * @brief Internal function to temporarily modify probabilities based on
   * received communication.
   * @param probabilities A float pointer to the state probability array to
   * influence.
   */
  void influence_probabilities(float* probabilities);

  /**
   * @brief Internal function to set the red and green LEDs depending on state.
   * Primarily used for debugging.
   * @param state The state to set the LEDs for.
   */
  void set_state_leds(StateEnum state);
};
