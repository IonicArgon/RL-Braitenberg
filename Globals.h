#pragma once
#include "mbed.h"

#ifndef NUM_STATES
// The maximum number of states.
#define NUM_STATES 5
#endif

#ifndef MAIL_SIZE
// The max size for a mail queue for transmission requests and incoming
// messages.
#define MAIL_SIZE 16
#endif

#ifndef MSG_SIZE
// The size of a message in bytes.
#define MSG_SIZE 32
#endif

/**
 * @brief Enum for possible states. Underlying type set to
 * `uint8_t` for proper packing of message struct.
 */
enum StateEnum : uint8_t {
  IDLE = 0,
  COWARD,
  AGGRESSIVE,
  LOVE,
  EXPLORER,
};

/**
 * @brief Struct to store light levels from LDR sensors.
 * @param lvl_left The value of the left LDR from 0.0 - 1.0 (inclusive).
 * @param lvl_right The value of the right LDR from 0.0 - 1.0 (inclusive).
 */
struct LightLevels {
  float lvl_left;
  float lvl_right;
};

/**
 * @brief Enum for possible directions to set motors to.
 */
enum Direction {
  FORWARD,
  REVERSE,
  STOP,
};

#pragma pack(push, 1)
/**
 * @brief Struct to store a message, either received or transmitted.
 * @note Packed using `#pragma pack`, guaranteeing a 32-byte struct.
 * @param prev_lvls A `LightLevels` struct containing light levels before
 * entering `prev_state`.
 * @param curr_lvls A `LightLevels` struct containing light levels after exiting
 * `prev_state`.
 * @param prev_state The `StateEnum` representing what state the vehicle was
 * previously in.
 * @param padding A 15-byte array for padding. Do not write data to this as
 * it'll be ignored.
 */
struct CommsMsg {
  LightLevels prev_lvls;
  LightLevels curr_lvls;
  StateEnum prev_state;
  uint8_t padding[15] = {0};
};
#pragma pack(pop)
