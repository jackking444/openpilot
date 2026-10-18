#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

extern bool enable_gas_interceptor;

#define MSG_LENKHILFE_3         0x0D0U   // RX from EPS, for steering angle and driver steering torque
#define MSG_HCA_1               0x0D2U   // TX by OP, Heading Control Assist steering torque
#define MSG_BREMSE_1            0x1A0U   // RX from ABS, for ego speed
#define MSG_MOTOR_2             0x288U   // RX from ECU, for CC state and brake switch state
#define MSG_ACC_SYSTEM          0x368U   // TX by OP, longitudinal acceleration controls
#define MSG_MOTOR_3             0x380U   // RX from ECU, for driver throttle input
#define MSG_GRA_NEU             0x38AU   // TX by OP, ACC control buttons for cancel/resume
#define MSG_MOTOR_5             0x480U   // RX from ECU, for ACC main switch state
#define MSG_ACC_GRA_ANZEIGE     0x56AU   // TX by OP, ACC HUD
#define MSG_LDW_1               0x5BEU   // TX by OP, Lane line recognition and text alerts
#define MSG_GAS_COMMAND         0x200U   // TX by OP, comma pedal command
#define MSG_GAS_SENSOR          0x201U   // RX from comma pedal

#define VOLKSWAGEN_PQ_PEDAL_RX_TIMEOUT_US 100000U
#define VOLKSWAGEN_PQ_SPEED_RX_TIMEOUT_US 100000U
#define VOLKSWAGEN_PQ_PEDAL_MAX_SPEED 1800U  // 18.00 km/h, raw 0.01 km/h units

static bool volkswagen_pq_stock_gas_pressed = false;
static bool volkswagen_pq_interceptor_gas_pressed = false;
static uint32_t volkswagen_pq_pedal_rx_timestamp = 0U;
static bool volkswagen_pq_pedal_rx_seen = false;
static uint32_t volkswagen_pq_speed_rx_timestamp = 0U;
static bool volkswagen_pq_speed_rx_seen = false;
static uint16_t volkswagen_pq_speed = 0U;

static uint8_t volkswagen_pedal_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFFU;
  for (int i = len - 1; i >= 0; i--) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0xD5U) : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

static uint16_t volkswagen_pedal_value(const CANPacket_t *msg, int offset) {
  return ((uint16_t)msg->data[offset] << 8U) | msg->data[offset + 1];
}

static bool volkswagen_pq_pedal_inputs_fresh(void) {
  uint32_t ts = microsecond_timer_get();
  bool pedal_fresh = volkswagen_pq_pedal_rx_seen &&
                     (safety_get_ts_elapsed(ts, volkswagen_pq_pedal_rx_timestamp) < VOLKSWAGEN_PQ_PEDAL_RX_TIMEOUT_US);
  bool speed_fresh = volkswagen_pq_speed_rx_seen &&
                     (safety_get_ts_elapsed(ts, volkswagen_pq_speed_rx_timestamp) < VOLKSWAGEN_PQ_SPEED_RX_TIMEOUT_US);
  return pedal_fresh && speed_fresh;
}

static bool volkswagen_pedal_tx_checks(const CANPacket_t *msg) {
  if (!enable_gas_interceptor || !volkswagen_longitudinal || GET_LEN(msg) != 6 ||
      msg->data[5] != volkswagen_pedal_crc8(msg->data, 5)) {
    return false;
  }

  uint16_t gas = volkswagen_pedal_value(msg, 0);
  uint16_t gas2 = volkswagen_pedal_value(msg, 2);
  bool enabled = (msg->data[4] & 0x80U) != 0U;

  if (!enabled) {
    return (gas == 0U) && (gas2 == 0U) && ((msg->data[4] & 0x70U) == 0U);
  }

  return get_longitudinal_allowed() && volkswagen_pq_pedal_inputs_fresh() &&
         volkswagen_pq_speed < VOLKSWAGEN_PQ_PEDAL_MAX_SPEED &&
         gas >= 430U && gas <= 1300U && gas2 == gas / 2U && (msg->data[4] & 0x70U) == 0U;
}

static uint32_t volkswagen_pq_get_checksum(const CANPacket_t *msg) {
  if (msg->addr == MSG_GAS_SENSOR) {
    return msg->data[5];
  }
  return (uint32_t)msg->data[(msg->addr == MSG_MOTOR_5) ? 7 : 0];
}

static uint8_t volkswagen_pq_get_counter(const CANPacket_t *msg) {
  uint8_t counter = 0U;

  if (msg->addr == MSG_LENKHILFE_3) {
    counter = (uint8_t)(msg->data[1] & 0xF0U) >> 4;
  } else if (msg->addr == MSG_GRA_NEU) {
    counter = (uint8_t)(msg->data[2] & 0xF0U) >> 4;
  } else if (msg->addr == MSG_GAS_SENSOR) {
    counter = (uint8_t)(msg->data[4] & 0x0FU);
  } else {
  }

  return counter;
}

static uint32_t volkswagen_pq_compute_checksum(const CANPacket_t *msg) {
  if (msg->addr == MSG_GAS_SENSOR) {
    return volkswagen_pedal_crc8(msg->data, 5);
  }
  int len = GET_LEN(msg);
  uint8_t checksum = 0U;
  int checksum_byte = (msg->addr == MSG_MOTOR_5) ? 7 : 0;

  // Simple XOR over the payload, except for the byte where the checksum lives.
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      checksum ^= (uint8_t)msg->data[i];
    }
  }

  return checksum;
}

static safety_config volkswagen_pq_init(uint16_t param) {
  alka_allowed = true;  // dp - ALKA enabled for VW PQ
  volkswagen_pq_stock_gas_pressed = false;
  volkswagen_pq_interceptor_gas_pressed = false;
  volkswagen_pq_pedal_rx_timestamp = 0U;
  volkswagen_pq_pedal_rx_seen = false;
  volkswagen_pq_speed_rx_timestamp = 0U;
  volkswagen_pq_speed_rx_seen = false;
  volkswagen_pq_speed = 0U;

  // Transmit of GRA_Neu is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_PQ_STOCK_TX_MSGS[] = {{MSG_HCA_1, 0, 5, .check_relay = true}, {MSG_LDW_1, 0, 8, .check_relay = true},
                                                {MSG_GRA_NEU, 0, 4, .check_relay = false}, {MSG_GRA_NEU, 2, 4, .check_relay = false}};

  static const CanMsg VOLKSWAGEN_PQ_LONG_TX_MSGS[] =  {{MSG_HCA_1, 0, 5, .check_relay = true}, {MSG_LDW_1, 0, 8, .check_relay = true},
                                                {MSG_ACC_SYSTEM, 0, 8, .check_relay = true}, {MSG_ACC_GRA_ANZEIGE, 0, 8, .check_relay = true},
                                                {MSG_GAS_COMMAND, 0, 6, .check_relay = false}};

  static RxCheck volkswagen_pq_rx_checks[] = {
    {.msg = {{MSG_LENKHILFE_3, 0, 6, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_BREMSE_1, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_3, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_5, 0, 8, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_NEU, 0, 4, 30U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GAS_SENSOR, 0, 6, 50U, .max_counter = 15U, .ignore_quality_flag = true}}},
  };

  volkswagen_common_init();

#ifdef ALLOW_DEBUG
  enable_gas_interceptor = GET_FLAG(param, FLAG_VOLKSWAGEN_GAS_INTERCEPTOR);
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif
  safety_config cfg = volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_pq_rx_checks, VOLKSWAGEN_PQ_LONG_TX_MSGS) :
                                                BUILD_SAFETY_CFG(volkswagen_pq_rx_checks, VOLKSWAGEN_PQ_STOCK_TX_MSGS);
  if (!enable_gas_interceptor) {
    cfg.rx_checks_len--;
  }
  return cfg;
}

static void volkswagen_pq_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state from speed value.
    // Signal: Bremse_1.BR1_Rad_kmh
    if (msg->addr == MSG_BREMSE_1) {
      volkswagen_pq_speed = (uint16_t)(((msg->data[2] & 0xFEU) >> 1) | (msg->data[3] << 7));
      volkswagen_pq_speed_rx_timestamp = microsecond_timer_get();
      volkswagen_pq_speed_rx_seen = true;
      vehicle_moving = volkswagen_pq_speed > 0U;
    }

    // Update driver input torque samples
    // Signal: Lenkhilfe_3.LH3_LM (absolute torque)
    // Signal: Lenkhilfe_3.LH3_LMSign (direction)
    if (msg->addr == MSG_LENKHILFE_3) {
      int torque_driver_new = msg->data[2] | ((msg->data[3] & 0x3U) << 8);
      int sign = (msg->data[3] & 0x4U) >> 2;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    // ACC main switch tracking - applies to both longitudinal and non-longitudinal modes
    // Signal: Motor_5.MO5_GRA_Hauptsch
    if (msg->addr == MSG_MOTOR_5) {
      acc_main_on = GET_BIT(msg, 50U);
      // dp - ALKA: use ACC main for LKAS state
      if (alka_allowed && (alternative_experience & ALT_EXP_ALKA)) {
        lkas_on = acc_main_on;
      }
      if (!acc_main_on) {
        controls_allowed = false;
      }
    }

    if (volkswagen_longitudinal) {
      if (msg->addr == MSG_GRA_NEU) {
        // If ACC main switch is on, enter controls on falling edge of Set or Resume
        // Signal: GRA_Neu.GRA_Neu_Setzen
        // Signal: GRA_Neu.GRA_Neu_Recall
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 17U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
        // Exit controls on rising edge of Cancel, override Set/Resume if present simultaneously
        // Signal: GRA_ACC_01.GRA_Abbrechen
        if (GET_BIT(msg, 9U)) {
          controls_allowed = false;
        }
      }
    } else {
      if (msg->addr == MSG_MOTOR_2) {
        // Enter controls on rising edge of stock ACC, exit controls if stock ACC disengages
        // Signal: Motor_2.MO2_Sta_GRA
        int acc_status = (msg->data[2] & 0xC0U) >> 6;
        bool cruise_engaged = (acc_status == 1) || (acc_status == 2);
        pcm_cruise_check(cruise_engaged);
      }
    }

    if ((msg->addr == MSG_GAS_SENSOR) && enable_gas_interceptor) {
      volkswagen_pq_pedal_rx_timestamp = microsecond_timer_get();
      volkswagen_pq_pedal_rx_seen = true;
      uint16_t gas1 = volkswagen_pedal_value(msg, 0);
      uint16_t gas2 = volkswagen_pedal_value(msg, 2);
      volkswagen_pq_interceptor_gas_pressed = ((uint32_t)gas1 + gas2) > 950U;
      gas_pressed = volkswagen_pq_interceptor_gas_pressed;
    }

    // Signal: Motor_3.MO3_Pedalwert
    if (msg->addr == MSG_MOTOR_3) {
      volkswagen_pq_stock_gas_pressed = !enable_gas_interceptor && (msg->data[2] != 0U);
      gas_pressed = enable_gas_interceptor ? volkswagen_pq_interceptor_gas_pressed : volkswagen_pq_stock_gas_pressed;
    }

    // Signal: Motor_2.MO2_BLS
    if (msg->addr == MSG_MOTOR_2) {
      brake_pressed = (msg->data[2] & 0x1U);
    }
  }
}

static bool volkswagen_pq_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits VOLKSWAGEN_PQ_STEERING_LIMITS = {
    .max_torque = 300,               // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
    .max_rt_delta = 113,             // 6 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 75 ; 125 * 1.5 for safety pad = 113
    .max_rate_up = 6,                // 3.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .max_rate_down = 10,             // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .driver_torque_multiplier = 3,
    .driver_torque_allowance = 80,
    .type = TorqueDriverLimited,
  };

  // longitudinal limits
  // acceleration in m/s2 * 1000 to avoid floating point math
  const LongitudinalLimits VOLKSWAGEN_PQ_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  };

  bool tx = true;

  // Safety check for HCA_1 Heading Control Assist torque
  // Signal: HCA_1.LM_Offset (absolute torque)
  // Signal: HCA_1.LM_Offsign (direction)
  if (msg->addr == MSG_HCA_1) {
    int desired_torque = msg->data[2] | ((msg->data[3] & 0x7FU) << 8);
    desired_torque = desired_torque / 32;  // DBC scale from PQ network to centi-Nm
    int sign = (msg->data[3] & 0x80U) >> 7;
    if (sign == 1) {
      desired_torque *= -1;
    }

    uint32_t hca_status = ((msg->data[1] >> 4) & 0xFU);
    bool steer_req = ((hca_status == 5U) || (hca_status == 7U));

    if (steer_torque_cmd_checks(desired_torque, steer_req, VOLKSWAGEN_PQ_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // Safety check for acceleration commands
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s2 boundaries
  if (msg->addr == MSG_ACC_SYSTEM) {
    // Signal: ACC_System.ACS_Sollbeschl (acceleration in m/s2, scale 0.005, offset -7.22)
    int desired_accel = ((((msg->data[4] & 0x7U) << 8) | msg->data[3]) * 5U) - 7220U;

    if (longitudinal_accel_checks(desired_accel, VOLKSWAGEN_PQ_LONG_LIMITS)) {
      tx = false;
    }
  }

  if ((msg->addr == MSG_GAS_COMMAND) && !volkswagen_pedal_tx_checks(msg)) {
    tx = false;
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_GRA_NEU) && !controls_allowed) {
    // Signal: GRA_Neu.GRA_Neu_Setzen
    // Signal: GRA_Neu.GRA_Neu_Recall
    if (GET_BIT(msg, 16U) || GET_BIT(msg, 17U)) {
      tx = false;
    }
  }

  return tx;
}

static bool volkswagen_pq_fwd_hook(int bus_num, int addr) {
  return (bus_num == 2) && (addr == MSG_GAS_COMMAND);
}

const safety_hooks volkswagen_pq_hooks = {
  .init = volkswagen_pq_init,
  .rx = volkswagen_pq_rx_hook,
  .tx = volkswagen_pq_tx_hook,
  .get_counter = volkswagen_pq_get_counter,
  .get_checksum = volkswagen_pq_get_checksum,
  .compute_checksum = volkswagen_pq_compute_checksum,
  .fwd = volkswagen_pq_fwd_hook,
};
