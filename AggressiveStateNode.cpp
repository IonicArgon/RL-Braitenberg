#include "AggressiveStateNode.h"

#include "VehicleContext.h"

void AggressiveStateNode::enter(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Entering Aggressive state\r\n");
#endif
  // Stop the motors before we start execution of Aggressive state.
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

void AggressiveStateNode::execute(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Executing Aggressive state\r\n");
#endif
  // Read the light sensors and use to change motor speed.
  // - Left speed is proportional to increasing light levels on right LDR.
  // - Right speed is proportional to increasing light levels on the left LDR.
  // Goal is to run towards light and speed up the brighter it is.
  LightLevels lvls = ctx.get_curr_light_lvls();
  float pwm_l = lvls.lvl_right * m_max_speed;
  float pwm_r = lvls.lvl_left * m_max_speed;
  ctx.set_motor_speeds(FORWARD, FORWARD, pwm_l, pwm_r);

  // After minimum time in state has elapsed, then transition to new state.
  if (ctx.get_elapsed_time_in_state() >= ctx.get_min_duration(get_enum())) {
    StateEnum next_state = ctx.sample_next_state();

    ctx.transition_to(next_state);
    return;
  }
}

void AggressiveStateNode::exit(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Exiting Aggressive state\r\n");
#endif
  // Stop the motors before a transition to another state.
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

StateEnum AggressiveStateNode::get_enum() const { return AGGRESSIVE; }
