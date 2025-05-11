#include "LoveStateNode.h"

#include "VehicleContext.h"

void LoveStateNode::enter(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Entering Love state\r\n");
#endif
  // Stop the motors before we start execution of the Love state.
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

void LoveStateNode::execute(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Executing Love state\r\n");
#endif
  // Read the light sensors and use to change motor speed.
  // - Left speed is inversely proportional to increasing light levels on right
  // LDR.
  // - Right speed is inversely proportional to increasing light levels on left
  // LDR. Goal is to run atowards light, slowing down the brighter it is.
  LightLevels lvls = ctx.get_curr_light_lvls();
  float pwm_l = 1.0 - (lvls.lvl_right * m_max_speed);
  float pwm_r = 1.0 - (lvls.lvl_left * m_max_speed);
  ctx.set_motor_speeds(FORWARD, FORWARD, pwm_l, pwm_r);

  // After minimum time in state has elapsed, then transition to new state.
  if (ctx.get_elapsed_time_in_state() >= ctx.get_min_duration(get_enum())) {
    StateEnum next_state = ctx.sample_next_state();

    ctx.transition_to(next_state);
    return;
  }
}

void LoveStateNode::exit(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Exiting Love state\r\n");
#endif
  // Stop the motors before a transition to another state.
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

StateEnum LoveStateNode::get_enum() const { return LOVE; }
