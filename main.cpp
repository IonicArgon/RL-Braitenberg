#include "CommsContext.h"
#include "VehicleContext.h"
#include "mbed.h"

// For seeding srand()
#define PIN_ENTROPY PF_4

// Set tick rates for each thread.
const auto FSM_TICK_RATE = 10ms;
const auto COMMS_TICK_RATE = 10ms;

// Set up the threads, entropy pin, and vehicle context.
VehicleContext vehicle_ctx(PC_1, PF_10, PC_0, PF_9, PF_5, PF_3, PF_1, PC_15,
                           PF_6, PA_3, PG_13, PG_14);
AnalogIn entropy(PIN_ENTROPY);
Thread thread_fsm;
Thread thread_comms;

// Main procedure for FSM
void fsm_proc() {
  while (true) {
    auto cycle_start = Kernel::Clock::now();

#ifdef PRINT_DEBUG
    printf("Running FSM tick\r\n");
#endif
    vehicle_ctx.run_fsm_cycle();

    auto cycle_end = Kernel::Clock::now();
    auto cycle_delta = cycle_end - cycle_start;

    // If the tick completes earlier than our tick rate, defer to the other
    // thread.
    if (cycle_delta < FSM_TICK_RATE) {
      ThisThread::sleep_for(FSM_TICK_RATE - cycle_delta);
    }
  }
}

// Main procedure for communication thread
void comms_proc() {
  while (true) {
    auto cycle_start = Kernel::Clock::now();

    vehicle_ctx.m_comms_ctx.run_comms_cycle();

    auto cycle_end = Kernel::Clock::now();
    auto cycle_delta = cycle_end - cycle_start;

    // If the tick completes earlier than our tick rate, defer to the other
    // thread.
    if (cycle_delta < COMMS_TICK_RATE) {
      ThisThread::sleep_for(COMMS_TICK_RATE - cycle_delta);
    }
  }
}

int main() {
  srand(entropy.read_u16());

  // If either the FSM or communication thread fails to initialize,
  // crash with an error.
  auto fsm_thread_start_status = thread_fsm.start(fsm_proc);
  if (fsm_thread_start_status != osOK) {
    error("Failed to start FSM thread\r\n");
  }
#ifdef PRINT_DEBUG
  printf("Initialized FSM thread\r\n");
#endif

  auto comms_thread_start_status = thread_comms.start(comms_proc);
  if (comms_thread_start_status != osOK) {
    error("Failed to start comms thread\r\n");
  }
#ifdef PRINT_DEBUG
  printf("Initialized comms thread\r\n");
#endif

  // And just defer forever.
  while (true) {
    ThisThread::sleep_for(5s);
  }

  return 0;
}
