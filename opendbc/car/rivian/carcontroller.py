import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_longitudinal, create_wheel_touch, create_adas_status
from opendbc.car.rivian.values import CarControllerParams
from opendbc.sunnypilot.car.rivian.mads import MadsCarController


class CarController(CarControllerBase, MadsCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.cancel_frames = 0

    # accel transition
    self.last_accel = 0

  def get_acceleration(self, CS, CC):
    # Stock Rivian ACC rate limits
    max_accel_rate = 0.23
    max_decel_rate = 0.64

    accel_op = CC.actuators.accel
    accel_stock = CS.acm_long_accel

    # When both want to brake: use OP's smoother deceleration with rate limiting
    if (accel_op < 0) and (accel_stock < 0):
      target_accel = accel_op

      # Soft braking gets lower rate limit for smoother feel
      if target_accel > -2.0:
        max_decel_rate = 0.15

      # Don't rate limit brake release
      if target_accel > self.last_accel:
        final_accel = target_accel
      else:
        final_accel = max(target_accel, self.last_accel - max_decel_rate)

        # When both want to accelerate: use openpilot's control
    elif (accel_op >= 0.2) and (accel_stock >= 0.2):
      final_accel = accel_op
    # When they disagree: stock acts as governor (respects true set-speed limit)
    else:
      final_accel = accel_stock

    # Apply rate limits to prevent abrupt changes
    final_accel = np.clip(final_accel, self.last_accel - max_decel_rate, self.last_accel + max_accel_rate)

    if CS.out.gasPressed or not CC.enabled:
      final_accel = 0

    return final_accel

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, CC, CC_SP, CS)
    actuators = CC.actuators
    can_sends = []

    apply_torque = 0
    steer_max = round(float(np.interp(CS.out.vEgoRaw, CarControllerParams.STEER_MAX_LOOKUP[0],
                                      CarControllerParams.STEER_MAX_LOOKUP[1])))
    if self.mads.lat_active:
      new_torque = int(round(CC.actuators.torque * steer_max))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams, steer_max)

    # send steering command
    self.apply_torque_last = apply_torque
    can_sends.append(create_lka_steering(self.packer, self.frame, CS.acm_lka_hba_cmd, apply_torque, CC.enabled, CC.latActive, self.mads))

    if self.frame % 5 == 0 and False:
      can_sends.append(create_wheel_touch(self.packer, CS.sccm_wheel_touch, CC.enabled))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      accel = self.get_acceleration(CS, CC)
      self.last_accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      can_sends.append(create_longitudinal(self.packer, self.frame, self.last_accel, CC.enabled))
    else:
      interface_status = None
      if CC.cruiseControl.cancel:
        # if there is a noEntry, we need to send a status of "available" before the ACM will accept "unavailable"
        # send "available" right away as the VDM itself takes a few frames to acknowledge
        interface_status = 1 if self.cancel_frames < 5 else 0
        self.cancel_frames += 1
      else:
        self.cancel_frames = 0

      can_sends.append(create_adas_status(self.packer, CS.vdm_adas_status, interface_status))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
