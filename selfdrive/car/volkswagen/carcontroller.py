from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip,interp
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    self.gra_acc_counter_last = None
    self.frame = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero.

      if CC.latActive:
        new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_steer_last == apply_steer:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_steer -= (1, -1)[apply_steer < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_enabled = abs(apply_steer) > 0
      else:
        hca_enabled = False
        apply_steer = 0

      if not hca_enabled:
        self.hca_frame_timer_running = 0

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_steer_last = apply_steer
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      #stopping = True if CC.longActive and CS.out.standstill and starting else stopping
      if self.CP.enableGasInterceptor and CS.out.vEgo <10:
        accel = clip(self.CP.stopAccel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive and starting else accel
        stopping = True if CC.longActive and starting else stopping

      #can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, CANBUS.pt, CS.acc_type, CC.longActive, accel,
      #                                                   acc_control, stopping, starting, CS.esp_hold_confirmation))
      if self.CP.enableGasInterceptor:
        self.gas = 0.0
        if CC.longActive and actuators.accel >0 and CS.out.vEgo <10:
          speed = CS.out.vEgo
          cd = 0.31
          frontalArea = 2.3
          drag = 0.5 * cd * frontalArea * (speed ** 2)

          mass = 1772
          g = 9.81
          rollingFrictionCoefficient = 0.02
          friction = mass * g * rollingFrictionCoefficient

          desiredAcceleration = actuators.accel
          acceleration = mass * desiredAcceleration

          driveTrainLosses = 0  # 600 for the engine, 200 for trans, low speed estimate
          powerNeeded = (drag + friction + acceleration) * speed + driveTrainLosses
          POWER_LOOKUP_BP = [0, 25000 * 1.6 / 2.6,
                               75000]  # 160NM@1500rpm=25kW but with boost, no boost means *1.6/2.6
          PEDAL_LOOKUP_BP = [227, 1772 * 0.4,
                               1772 * 100 / 140]  # Not max gas, max gas gives 140hp, we want at most 100 hp, also 40% throttle might prevent an upshift

          GAS_MULTIPLIER_BP = [0, 0.1, 0.2, 0.4, 8.3]
          GAS_MULTIPLIER_V = [1.15, 1.15, 1.2, 1.25, 1.]

          powerNeeded_mult = interp(CS.out.vEgo, [20 / 3.6, 40 / 3.6], [2, 1])
          powerNeeded = int(round(powerNeeded * powerNeeded_mult))
          #apply_gas = int(round(interp(powerNeeded, POWER_LOOKUP_BP, PEDAL_LOOKUP_BP)))
          #apply_gas = int(round(apply_gas * int(round(interp(speed, GAS_MULTIPLIER_BP, GAS_MULTIPLIER_V)))))
          #wind_brake = interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15])
          #gas_mult = interp(CS.out.vEgo, [0., 10.], [0.4, 1.0])
          PEDAL_SCALE = interp(CS.out.vEgo, [0.0, 3, 5], [0.4, 0.5, 0.0])
          # offset for creep and windbrake
          pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, 5], [-.4, 0.0, 0.2])
          pedal_command = 430 +PEDAL_SCALE * (actuators.accel + pedal_offset)*(1200-430)
          interceptor_gas_cmd = clip(pedal_command, 420, 1200)
          self.gas = interceptor_gas_cmd
          #self.gas = apply_gas if apply_gas < 1200  else 1200
        else:
          self.gas = 0.0
        can_sends.append(self.CCS.create_pedal_control(self.packer_pt,  CANBUS.pt, self.gas, self.frame // 2))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
