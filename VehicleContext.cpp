#include "VehicleContext.h"

#include "CommsContext.h"

VehicleContext::VehicleContext(PinName ldr_l, PinName ldr_r, PinName ldr_l_gnd,
                               PinName ldr_r_gnd, PinName mtr_l_in1,
                               PinName mtr_l_in2, PinName mtr_r_in3,
                               PinName mtr_r_in4, PinName mtr_l_pwm,
                               PinName mtr_r_pwm, PinName led_g, PinName led_r,
                               float learning_rate, float ci_change_rate)
    :
#ifdef VEHICLE_1
      m_comms_ctx(PE_14, PE_13, PE_12, PE_11, PE_9, 0x1111111111, 0x0000000000),
#else
      m_comms_ctx(PE_14, PE_13, PE_12, PE_11, PE_9, 0x0000000000, 0x1111111111),
#endif
      m_ldr_l(ldr_l),
      m_ldr_r(ldr_r),
      m_ldr_l_gnd(ldr_l_gnd, 0),
      m_ldr_r_gnd(ldr_r_gnd, 0),
      m_mtr_l_in1(mtr_l_in1, 0),
      m_mtr_l_in2(mtr_l_in2, 0),
      m_mtr_l_pwm(mtr_l_pwm),
      m_mtr_r_in3(mtr_r_in3, 0),
      m_mtr_r_in4(mtr_r_in4, 0),
      m_mtr_r_pwm(mtr_r_pwm),
      m_curr_state_ptr(nullptr),
      m_curr_state(IDLE),
      m_prev_state(IDLE),
      m_light_lvl_min({1.0, 1.0}),
      m_light_lvl_max({0.0, 0.0}),
      m_comms_influence(0.0f),
      m_learning_rate(learning_rate),
      m_ci_change_rate(ci_change_rate),
      m_g_led(led_g),
      m_r_led(led_r) {
  // Set up photoresistors
  m_ldr_l.set_reference_voltage(3.0f);
  m_ldr_r.set_reference_voltage(3.0f);

  // Initialize PWM for drive
  m_mtr_l_pwm.period(0.00005f);  // 1 kHz
  m_mtr_r_pwm.period(0.00005f);
  m_mtr_l_pwm.write(0.0f);
  m_mtr_r_pwm.write(0.0f);

  // Set up pointers to state objects
  m_state_node_instances[IDLE] = &m_state_idle;
  m_state_node_instances[COWARD] = &m_state_coward;
  m_state_node_instances[AGGRESSIVE] = &m_state_aggressive;
  m_state_node_instances[LOVE] = &m_state_love;
  m_state_node_instances[EXPLORER] = &m_state_explorer;

  // And initialize the FSM
  initialize_fsm();
}

void VehicleContext::initialize_fsm(void) {
  // Default the probabilities to a uniform distribution
  for (int i = 0; i < NUM_STATES; ++i) {
    for (int j = 0; j < NUM_STATES; ++j) {
      m_probability_table[i][j] = 1.0f / static_cast<float>(NUM_STATES);
    }
  }

  // Then grab the first state note (defaults to IDLE per constructor)
  m_curr_state_ptr = get_state_node(m_curr_state);

  // Grab the entry time to use for tick update later
  m_time_state_entry = Kernel::Clock::now();

  // Read the light sensors and record the entry light level for reward
  // calculations later
  read_sensors();
  m_light_lvl_entry = m_light_lvl_curr;

  // Then run the "enter" function for our first state
  if (m_curr_state_ptr) {
    m_curr_state_ptr->enter(*this);
  }
}

StateNode* VehicleContext::get_state_node(StateEnum state) const {
  return m_state_node_instances[state];
}

void VehicleContext::read_sensors(void) {
  // Grab raw values
  float raw_ldr_l = m_ldr_l.read();
  float raw_ldr_r = m_ldr_r.read();

  // Then get the mins and maxes for normalization
  float min_ldr_l = min(raw_ldr_l, m_light_lvl_min.lvl_left);
  float max_ldr_l = max(raw_ldr_l, m_light_lvl_max.lvl_left);
  float min_ldr_r = min(raw_ldr_r, m_light_lvl_min.lvl_right);
  float max_ldr_r = max(raw_ldr_r, m_light_lvl_max.lvl_right);

  // Save the mins and maxes for later use
  m_light_lvl_min = {
      .lvl_left = min_ldr_l,
      .lvl_right = min_ldr_r,
  };
  m_light_lvl_max = {
      .lvl_left = max_ldr_l,
      .lvl_right = max_ldr_r,
  };

  // Normalize the light sensor values
  float norm_ldr_l = (raw_ldr_l - m_light_lvl_min.lvl_left) /
                     (m_light_lvl_max.lvl_left - m_light_lvl_min.lvl_left);
  float norm_ldr_r = (raw_ldr_r - m_light_lvl_min.lvl_right) /
                     (m_light_lvl_max.lvl_right - m_light_lvl_min.lvl_right);

  // And write the normalized values to m_light_lvl_curr
  m_light_lvl_curr = {
      .lvl_left = norm_ldr_l,
      .lvl_right = norm_ldr_r,
  };
}

void VehicleContext::run_fsm_cycle(void) {
  // Read the light sensors on every tick of the FSM cycle
  read_sensors();

  if (m_curr_state_ptr) {
    // And execute the procedure for the state
    m_curr_state_ptr->execute(*this);
  } else {
    // We shouldn't be missing the state pointer
    // If we are, default to IDLE
    transition_to(IDLE);
  }
}

void VehicleContext::transition_to(StateEnum next_state) {
  if (next_state >= NUM_STATES || next_state < 0) {
    // This should always be true, but in the off-chance, do not
    // attempt to transition to an invalid state
    return;
  }

  // Calculate our reward for previous state and update the appropriate
  // probability table
  float reward = calculate_reward(m_light_lvl_entry, m_light_lvl_curr);
  update_probability_table(reward);

  // Then run cleanup for the previous state
  if (m_curr_state_ptr) {
    m_curr_state_ptr->exit(*this);
  }

  // Now set the next state and pointer
  m_prev_state = m_curr_state;
  m_curr_state = next_state;
  m_curr_state_ptr = get_state_node(m_curr_state);

  // 33% chance we send a message about our previous state to the other vehicle
  CommsMsg msg = {
      .prev_lvls = m_light_lvl_entry,
      .curr_lvls = m_light_lvl_curr,
      .prev_state = m_prev_state,
  };
  if (rand() % 3 == 0) {
    if (!m_comms_ctx.try_queue_send(msg)) {
#ifdef PRINT_DEBUG
      printf("Could not send message\r\n");
#endif
    }
  }

  // Now transition into the new state, similar procedure to
  // initialize_fsm
  if (m_curr_state_ptr) {
    m_time_state_entry = Kernel::Clock::now();
    m_light_lvl_entry = m_light_lvl_curr;
    set_state_leds(m_curr_state);
    m_curr_state_ptr->enter(*this);
  } else {
    // Something went terribly wrong, so try to default to IDLE
    m_curr_state = IDLE;
    m_curr_state_ptr = get_state_node(IDLE);
    if (m_curr_state_ptr) {
      m_curr_state_ptr->enter(*this);
    }
  }
}

float VehicleContext::calculate_reward(LightLevels before, LightLevels after) {
  // The reward is based on average light level before and after the previous
  // state
  float avg_before = (before.lvl_left + before.lvl_right) / 2.0f;
  float avg_after = (after.lvl_left + after.lvl_right) / 2.0f;
  return avg_before - avg_after;
}

void VehicleContext::update_probability_table(float reward) {
  // Reward the previous state if our light levels decreased
  // Punish otherwise
  float& prob_to_update = m_probability_table[m_prev_state][m_curr_state];
  float delta = m_learning_rate * reward;
  prob_to_update += delta;

  // And modulate the probability of the other states to keep the sum at 1.0
  if (NUM_STATES > 1) {
    float reverse_delta = -delta / static_cast<float>(NUM_STATES - 1);
    for (int i = 0; i < NUM_STATES; ++i) {
      if (i != m_curr_state) {
        m_probability_table[m_prev_state][i] += reverse_delta;
      }
    }
  }

  normalize_probabilities(m_prev_state);
}

void VehicleContext::normalize_probabilities(StateEnum state) {
  const float min_prob = 0.01f;
  float sum = 0.0f;

  // Ensure no probability is below 0.01, then sum the values
  for (int i = 0; i < NUM_STATES; ++i) {
    if (m_probability_table[state][i] < min_prob) {
      m_probability_table[state][i] = min_prob;
    }
    sum += m_probability_table[state][i];
  }

  // And use the probability sum to normalize the values
  if (sum > 0.0f) {
    for (int i = 0; i < NUM_STATES; ++i) {
      m_probability_table[state][i] /= sum;
    }
  } else {
    // Something went terribly wrong, so we should reset to uniform
    // probabilities
    for (int i = 0; i < NUM_STATES; ++i) {
      m_probability_table[state][i] = 1.0f / static_cast<float>(NUM_STATES);
    }
  }
}

StateEnum VehicleContext::sample_next_state(void) {
  // Create a copy of the current state's probability array
  // so that we can...
  float probabilities[NUM_STATES];
  for (int i = 0; i < NUM_STATES; ++i) {
    probabilities[i] = m_probability_table[m_curr_state][i];
  }

#ifdef PRINT_DEBUG
  printf("Before comms influence\r\n");
  for (int i = 0; i < NUM_STATES; ++i) {
    printf("State %d | Probability: %f | ", i, probabilities[i]);
  }
  printf("\r\n\r\n");
#endif

  // ...temporarily influence the probabilities
  influence_probabilities(probabilities);

#ifdef PRINT_DEBUG
  printf("After comms influence\r\n");
  for (int i = 0; i < NUM_STATES; ++i) {
    printf("State %d | Probability: %f | ", i, probabilities[i]);
  }
  printf("\r\n\r\n");
#endif

  float sample = ((float)rand() / (float)(RAND_MAX));

  // Use a cummulative sum to determine what state to enter into next
  float cum_sum = 0.0f;
  for (int i = 0; i < NUM_STATES; ++i) {
    cum_sum += probabilities[i];
    if (sample <= cum_sum) {
      return static_cast<StateEnum>(i);
    }
  }

  // Otherwise we fallback to IDLE
  return IDLE;
}

LightLevels VehicleContext::get_curr_light_lvls(void) const {
  return m_light_lvl_curr;
}

Kernel::Clock::duration VehicleContext::get_elapsed_time_in_state(void) const {
  return Kernel::Clock::now() - m_time_state_entry;
}

Kernel::Clock::duration VehicleContext::get_min_duration(
    StateEnum state) const {
  return m_min_state_duration[state];
}

void VehicleContext::set_motor_speeds(Direction dir_l, Direction dir_r,
                                      float pwm_l, float pwm_r) {
  switch (dir_l) {
    case FORWARD:
      m_mtr_l_in1.write(1);
      m_mtr_l_in2.write(0);
      break;
    case REVERSE:
      m_mtr_l_in1.write(0);
      m_mtr_l_in2.write(1);
      break;
    case STOP:
    default:
      m_mtr_l_in1.write(0);
      m_mtr_l_in2.write(0);
      break;
  }
  m_mtr_l_pwm.write(pwm_l);

  switch (dir_r) {
    case FORWARD:
      m_mtr_r_in3.write(0);
      m_mtr_r_in4.write(1);
      break;
    case REVERSE:
      m_mtr_r_in3.write(1);
      m_mtr_r_in4.write(0);
      break;
    case STOP:
    default:
      m_mtr_r_in3.write(0);
      m_mtr_r_in4.write(0);
      break;
  }
  m_mtr_r_pwm.write(pwm_r);
}

void VehicleContext::influence_probabilities(float* probabilities) {
  // We only attempt to influence probabilities if there exists a comms message
  CommsMsg possible_msg;
  if (!m_comms_ctx.try_read(&possible_msg)) {
    return;
  } else {
#ifdef PRINT_DEBUG
    printf("Read incoming message, attempting to influenece\r\n");
#endif
  }

  // Compute the average difference before and after the previous state
  // the other vehicle was in
  float ldr_l_delta =
      possible_msg.curr_lvls.lvl_left - possible_msg.prev_lvls.lvl_left;
  float ldr_r_delta =
      possible_msg.curr_lvls.lvl_right - possible_msg.prev_lvls.lvl_right;
  float ldr_delta_avg = (ldr_l_delta + ldr_r_delta) / 2.0f;

  // A positive difference indicates increasing light level, so temporarily
  // decrease the chance we enter into the same state the other vehicle was just
  // in
  if (ldr_delta_avg > 0) {
    probabilities[possible_msg.prev_state] -= 0.2f;
    for (int i = 0; i < NUM_STATES; ++i) {
      if (i != possible_msg.prev_state) {
        probabilities[i] += (0.2f / (NUM_STATES - 1));
      }
    }
  } else {
    // A negative difference indicates increasing light level, so
    // increase the chance we enter the same state the other vehicle was just in
    probabilities[possible_msg.prev_state] += 0.2f;
    for (int i = 0; i < NUM_STATES; ++i) {
      if (i != possible_msg.prev_state) {
        probabilities[i] -= (0.2f / (NUM_STATES - 1));
      }
    }
  }

  // And normalize the values just in case
  const float min_prob = 0.01f;
  float sum = 0.0f;

  for (int i = 0; i < NUM_STATES; ++i) {
    if (probabilities[i] < min_prob) {
      probabilities[i] = min_prob;
    }
    sum += probabilities[i];
  }

  if (sum > 0.0f) {
    for (int i = 0; i < NUM_STATES; ++i) {
      probabilities[i] /= sum;
    }
  } else {
    // something went wrong updating weights, so reset to uniform
    for (int i = 0; i < NUM_STATES; ++i) {
      probabilities[i] = 1.0f / static_cast<float>(NUM_STATES);
    }
  }
}

void VehicleContext::set_state_leds(StateEnum state) {
  switch (state) {
    case IDLE:
    case EXPLORER:
      // IDLE or EXPLORER has both LEDs off
      // You can tell the difference between states based on whether the vehicle
      // moves or not
      m_g_led.write(0);
      m_r_led.write(0);
      break;
    case AGGRESSIVE:
      // Red LED only
      m_g_led.write(0);
      m_r_led.write(1);
      break;
    case COWARD:
      // Green LED only
      m_g_led.write(1);
      m_r_led.write(0);
      break;
    case LOVE:
      // And both LEDS
      m_g_led.write(1);
      m_r_led.write(1);
      break;
  }
}
