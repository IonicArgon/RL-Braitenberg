#include "IdleStateNode.h"

#include "VehicleContext.h"

void IdleStateNode::enter(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Entering Idle state\r\n");
#endif
  // Stop the motors before we start execution of the Idle state.
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

void IdleStateNode::execute(VehicleContext& ctx) {
  // In this state, we do nothing.
#ifdef PRINT_DEBUG
  printf("Executing Idle state\r\n");
#endif

  // After minimum time in state has elapsed, transition to new state.
  if (ctx.get_elapsed_time_in_state() >= ctx.get_min_duration(get_enum())) {
    StateEnum next_state = ctx.sample_next_state();

    ctx.transition_to(next_state);
    return;
  }
}

void IdleStateNode::exit(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Exiting Idle state\r\n");
#endif
  // Stop the motors before a transition to another state.
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

StateEnum IdleStateNode::get_enum() const { return IDLE; }
