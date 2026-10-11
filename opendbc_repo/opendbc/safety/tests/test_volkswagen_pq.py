#!/usr/bin/env python3
import unittest

from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.pqcan import crc8_pedal
from opendbc.car.volkswagen.pqcan import create_pedal_control
from opendbc.can import CANParser
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MSG_LENKHILFE_3 = 0x0D0       # RX from EPS, for steering angle and driver steering torque
MSG_HCA_1 = 0x0D2             # TX by OP, Heading Control Assist steering torque
MSG_BREMSE_1 = 0x1A0          # RX from ABS, for ego speed
MSG_MOTOR_2 = 0x288           # RX from ECU, for CC state and brake switch state
MSG_ACC_SYSTEM = 0x368        # TX by OP, longitudinal acceleration controls
MSG_MOTOR_3 = 0x380           # RX from ECU, for driver throttle input
MSG_GRA_NEU = 0x38A           # TX by OP, ACC control buttons for cancel/resume
MSG_MOTOR_5 = 0x480           # RX from ECU, for ACC main switch state
MSG_ACC_GRA_ANZEIGE = 0x56A   # TX by OP, ACC HUD
MSG_LDW_1 = 0x5BE             # TX by OP, Lane line recognition and text alerts


class TestVolkswagenPqSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  cruise_engaged = False

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_1, MSG_LDW_1)}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 113

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  # Ego speed (Bremse_1)
  def _speed_msg(self, speed):
    values = {"BR1_Rad_kmh": speed}
    return self.packer.make_can_msg_safety("Bremse_1", 0, values)

  # Brake light switch (shared message Motor_2)
  def _user_brake_msg(self, brake):
    # since this signal is used for engagement status, preserve current state
    return self._motor_2_msg(brake_pressed=brake, cruise_engaged=self.safety.get_controls_allowed())

  # ACC engaged status (shared message Motor_2)
  def _pcm_status_msg(self, enable):
    self.__class__.cruise_engaged = enable
    return self._motor_2_msg(cruise_engaged=enable)

  # Acceleration request to drivetrain coordinator
  def _accel_msg(self, accel):
    values = {"ACS_Sollbeschl": accel}
    return self.packer.make_can_msg_safety("ACC_System", 0, values)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"LH3_LM": abs(torque), "LH3_LMSign": torque < 0}
    return self.packer.make_can_msg_safety("Lenkhilfe_3", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1, hca_status=7):
    values = {"LM_Offset": abs(torque), "LM_OffSign": torque < 0, "HCA_Status": hca_status if steer_req else 3}
    return self.packer.make_can_msg_safety("HCA_1", 0, values)

  # ACC engagement and brake light switch status
  # Called indirectly for compatibility with common.py tests
  def _motor_2_msg(self, brake_pressed=False, cruise_engaged=False):
    values = {"MO2_BLS": brake_pressed,
              "MO2_Sta_GRA": cruise_engaged}
    return self.packer.make_can_msg_safety("Motor_2", 0, values)

  # ACC main switch status
  def _motor_5_msg(self, main_switch=False):
    values = {"MO5_GRA_Hauptsch": main_switch}
    return self.packer.make_can_msg_safety("Motor_5", 0, values)

  # Driver throttle input (Motor_3)
  def _user_gas_msg(self, gas):
    values = {"MO3_Pedalwert": gas}
    return self.packer.make_can_msg_safety("Motor_3", 0, values)

  # Cruise control buttons (GRA_Neu)
  def _button_msg(self, _set=False, resume=False, cancel=False, bus=2):
    values = {"GRA_Neu_Setzen": _set, "GRA_Recall": resume, "GRA_Abbrechen": cancel}
    return self.packer.make_can_msg_safety("GRA_Neu", bus, values)

  def test_torque_measurements(self):
    # TODO: make this test work with all cars
    self._rx(self._torque_driver_msg(50))
    self._rx(self._torque_driver_msg(-50))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())


class TestVolkswagenPqStockSafety(TestVolkswagenPqSafetyBase):
  # Transmit of GRA_Neu is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  TX_MSGS = [[MSG_HCA_1, 0], [MSG_GRA_NEU, 0], [MSG_GRA_NEU, 2], [MSG_LDW_1, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_1, MSG_LDW_1, 0x200]}

  def setUp(self):
    self.packer = CANPackerSafety("vw_pq")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))
    self.assertFalse(self._tx(self._button_msg(_set=True)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestVolkswagenPqLongSafety(TestVolkswagenPqSafetyBase, common.LongitudinalAccelSafetyTest):
  TX_MSGS = [[MSG_HCA_1, 0], [MSG_LDW_1, 0], [MSG_ACC_SYSTEM, 0], [MSG_ACC_GRA_ANZEIGE, 0], [0x200, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE, 0x200]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE)}
  INACTIVE_ACCEL = 3.01

  def setUp(self):
    self.packer = CANPackerSafety("vw_pq")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq,
                                 VolkswagenSafetyFlags.LONG_CONTROL | VolkswagenSafetyFlags.GAS_INTERCEPTOR)
    self.safety.init_tests()

  def test_pedal_command_checksum_and_dbc_roundtrip(self):
    msg = create_pedal_control(self.packer, 0, 1000, 3)
    self.assertEqual(msg[1][5], crc8_pedal(msg[1][:5]))

    parser = CANParser("vw_pq", [("GAS_COMMAND", 0)], 0)
    parser.update([(0, [msg])])
    self.assertEqual(1000, parser.vl["GAS_COMMAND"]["GAS_COMMAND"])
    self.assertEqual(500, parser.vl["GAS_COMMAND"]["GAS_COMMAND2"])
    self.assertEqual(1, parser.vl["GAS_COMMAND"]["ENABLE"])

  def _pedal_msg(self, gas=0, gas2=0, enabled=False, counter=0, checksum_ok=True):
    values = {"GAS_COMMAND": gas, "GAS_COMMAND2": gas2, "ENABLE": enabled, "COUNTER_PEDAL": counter}
    msg = self.packer.make_can_msg("GAS_COMMAND", 0, values)
    data = bytearray(msg[1])
    data[5] = crc8_pedal(data[:5])
    if not checksum_ok:
      data[5] ^= 0xFF
    return common.make_msg(0, 0x200, 6, data)

  def _pedal_sensor_msg(self, gas=0, gas2=0, state=0, counter=0, checksum_ok=True):
    values = {"INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas2, "STATE": state, "COUNTER_PEDAL": counter}
    msg = self.packer.make_can_msg("GAS_SENSOR", 0, values)
    data = bytearray(msg[1])
    data[5] = crc8_pedal(data[:5])
    if not checksum_ok:
      data[5] ^= 0xFF
    return common.make_msg(0, 0x201, 6, data)

  def test_pedal_tx_requires_flag_and_valid_bounded_payload(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._pedal_msg(1000, 500, True)))
    self.assertFalse(self._tx(self._pedal_msg(1301, 650, True)))
    self.assertFalse(self._tx(self._pedal_msg(1000, 499, True)))
    self.assertFalse(self._tx(self._pedal_msg(1000, 500, True, checksum_ok=False)))

    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._pedal_msg(1000, 500, True)))
    self.assertFalse(self._tx(self._pedal_msg(0, 500, False)))
    self.assertTrue(self._tx(self._pedal_msg()))

    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._pedal_msg(1000, 500, True)))

  def test_pedal_rx_checksum_counter_and_fault_state(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(self._pedal_sensor_msg(gas=500, gas2=500, counter=1)))
    self.assertFalse(self.safety.get_longitudinal_allowed())

    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self._rx(self._pedal_sensor_msg(counter=0, checksum_ok=False))
    self.assertFalse(self.safety.get_controls_allowed())

    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(self._pedal_sensor_msg(state=5, counter=0)))
    self.assertFalse(self.safety.get_controls_allowed())

    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    for counter in [0, 0, 0, 0, 0, 0]:
      self._rx(self._pedal_sensor_msg(counter=counter))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pedal_rx_is_not_required_without_interceptor(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, VolkswagenSafetyFlags.LONG_CONTROL)
    self.assertEqual(6, self.safety.get_current_safety_rx_checks_len())

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      # ACC main switch must be on, engage on falling edge
      self.safety.set_controls_allowed(0)
      self._rx(self._motor_5_msg(main_switch=False))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self._rx(self._button_msg(bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._motor_5_msg(main_switch=True))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._button_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    # Disable as soon as main switch turns off
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._motor_5_msg(main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_torque_cmd_enable_variants(self):
    # The EPS rack accepts either 5 or 7 for an enabled status, with different low speed tuning behavior
    self.safety.set_controls_allowed(1)
    for enabled_status in (5, 7):
      self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP, steer_req=1, hca_status=enabled_status)),
                      f"torque cmd rejected with {enabled_status=}")


if __name__ == "__main__":
  unittest.main()
