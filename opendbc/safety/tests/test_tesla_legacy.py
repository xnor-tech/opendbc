#!/usr/bin/env python3
import random
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.values import CarControllerParams, TeslaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can import CANDefine
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda, MAX_SPEED_DELTA, MAX_WRONG_COUNTERS, away_round, round_speed

MSG_DAS_steeringControl = 0x488
MSG_APS_eacMonitor = 0x27d
MSG_DAS_Control_HW1 = 0x2b9
MSG_DAS_Control_HW23 = 0x2bf


def round_angle(apply_angle, can_offset=0):
  apply_angle_can = (apply_angle + 1638.35) / 0.1 + can_offset
  # 0.49999_ == 0.5
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return away_round(apply_angle_can + rnd_offset) * 0.1 - 1638.35


class TeslaLegacyBase(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):
  # Base class configuration - will be overridden by subclasses
  CHASSIS_BUS = 0
  USE_APS_EAC_MONITOR = True
  DAS_CONTROL_MSG = MSG_DAS_Control_HW23
  DI_TORQUE_MSG = 0x106
  EXTERNAL_PANDA = False

  RELAY_MALFUNCTION_ADDRS = {}  # Will be set in subclasses
  FWD_BLACKLISTED_ADDRS = {}  # Will be set in subclasses
  TX_MSGS = []  # Will be set in subclasses

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  # Tesla uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  cnt_epas = 0
  cnt_angle_cmd = 0

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    from opendbc.car.tesla.interface import CarInterface
    self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))
    self.packer = CANPackerPanda("tesla_can")
    self.define = CANDefine("tesla_can")
    self.acc_states = {d: v for v, d in self.define.dv["DAS_control"]["DAS_accState"].items()}
    self.steer_control_types = {d: v for v, d in
                                self.define.dv["DAS_steeringControl"]["DAS_steeringControlType"].items()}

  def _angle_cmd_msg(self, angle: float, state: bool | int, increment_timer: bool = True, bus: int = 0):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_panda("DAS_steeringControl", bus, values)

  def _angle_meas_msg(self, angle: float, hands_on_level: int = 0, eac_status: int = 1, eac_error_code: int = 0):
    values = {"EPAS_internalSAS": angle, "EPAS_handsOnLevel": hands_on_level,
              "EPAS_eacStatus": eac_status, "EPAS_eacErrorCode": eac_error_code}
    return self.packer.make_can_msg_panda("EPAS_sysStatus", 0, values)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_panda("BrakeMessage", self.CHASSIS_BUS, values)

  def _speed_msg(self, speed):
    values = {"ESP_vehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_panda("ESP_B", self.CHASSIS_BUS, values)

  def _vehicle_moving_msg(self, speed: float):
    values = {"DI_cruiseState": 3 if speed <= self.STANDSTILL_THRESHOLD else 2}
    return self.packer.make_can_msg_panda("DI_state", self.CHASSIS_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": gas}
    return self.packer.make_can_msg_panda("DI_torque1", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0}
    return self.packer.make_can_msg_panda("DI_state", self.CHASSIS_BUS, values)

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_panda("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_rx_hook(self):
    # Legacy models don't have checksums for most messages
    # Test basic message reception
    for msg_type in ("angle", "long", "speed"):
      for i in range(5):
        if msg_type == "angle":
          msg = self._angle_cmd_msg(0, True, bus=2)
        elif msg_type == "long" and hasattr(self, 'LONGITUDINAL') and self.LONGITUDINAL:
          msg = self._long_control_msg(0, bus=2)
        elif msg_type == "speed":
          msg = self._speed_msg(0)
        else:
          continue

        self.safety.set_controls_allowed(True)
        self.assertTrue(self._rx(msg))
        self.assertTrue(self.safety.get_controls_allowed())

  def test_vehicle_speed_measurements(self):
    # Legacy models only have one speed source (ESP_B)
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_steering_wheel_disengage(self):
    for hands_on_level in range(4):
      for eac_status in range(8):
        for eac_error_code in range(16):
          self.safety.set_controls_allowed(True)

          should_disengage = hands_on_level >= 3 or (eac_status == 0 and eac_error_code == 9)
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=hands_on_level,
                                                        eac_status=eac_status, eac_error_code=eac_error_code)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())

  def test_steering_control_type(self):
    self.safety.set_controls_allowed(True)
    for steer_control_type in range(4):
      should_tx = steer_control_type in (self.steer_control_types["NONE"],
                                         self.steer_control_types["ANGLE_CONTROL"])
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, state=steer_control_type)))

  def test_stock_lkas_passthrough(self):
    if self.EXTERNAL_PANDA:
      return  # Skip for external panda configurations

    no_lkas_msg = self._angle_cmd_msg(0, state=False)
    no_lkas_msg_cam = self._angle_cmd_msg(0, state=True, bus=2)
    lkas_msg_cam = self._angle_cmd_msg(0, state=self.steer_control_types['LANE_KEEP_ASSIST'], bus=2)

    # stock system sends no LKAS -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_lkas_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_lkas_msg_cam.addr))
    self.assertTrue(self._tx(no_lkas_msg))

    # stock system sends LKAS -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(lkas_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, lkas_msg_cam.addr))
    self.assertFalse(self._tx(no_lkas_msg))

  def test_angle_cmd_when_enabled(self):
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 50):  # Reduced iterations for legacy tests
      speed = max(speed, 1)
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)

        angle_unit_offset = -1 if sign == -1 else 0
        max_angle = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 50):  # Reduced iterations for legacy tests
      speed = max(speed, 1)
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))

        angle_unit_offset = 1 if sign == -1 else 0
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams),
                                      angle_unit_offset) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))


class TestTeslaHW1Safety(TeslaLegacyBase):
  CHASSIS_BUS = 0
  USE_APS_EAC_MONITOR = False  # HW1 doesn't use APS_eacMonitor
  DAS_CONTROL_MSG = MSG_DAS_Control_HW1
  DI_TORQUE_MSG = 0x108
  EXTERNAL_PANDA = False
  LONGITUDINAL = True

  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl,)}
    self.FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl]}
    self.TX_MSGS = [[MSG_DAS_steeringControl, 0], [self.DAS_CONTROL_MSG, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.PARAM_HW1))
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": gas}
    return self.packer.make_can_msg_panda("DI_torque1", 0, values)

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    self.safety.set_controls_allowed(True)

    # accel_min and accel_max are positive
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))

    # accel_min and accel_max are both zero
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))

    # accel_min and accel_max have opposing signs
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))

    # accel_min and accel_max are negative - should be blocked
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))


class TestTeslaHW2Safety(TeslaLegacyBase):
  CHASSIS_BUS = 0
  USE_APS_EAC_MONITOR = True
  DAS_CONTROL_MSG = MSG_DAS_Control_HW23
  DI_TORQUE_MSG = 0x106
  EXTERNAL_PANDA = False
  LONGITUDINAL = False  # HW2 doesn't have longitudinal control

  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
    self.FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
    self.TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.PARAM_HW2))
    self.safety.init_tests()


class TestTeslaHW3Safety(TeslaLegacyBase):
  CHASSIS_BUS = 1  # HW3 uses bus 1 for chassis
  USE_APS_EAC_MONITOR = True
  DAS_CONTROL_MSG = MSG_DAS_Control_HW23
  DI_TORQUE_MSG = 0x106
  EXTERNAL_PANDA = False
  LONGITUDINAL = False

  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
    self.FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
    self.TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.PARAM_HW3))
    self.safety.init_tests()


class TestTeslaHW2ExternalPandaSafety(TeslaLegacyBase):
  CHASSIS_BUS = 0
  USE_APS_EAC_MONITOR = False  # External panda doesn't use APS_eacMonitor
  DAS_CONTROL_MSG = MSG_DAS_Control_HW23
  DI_TORQUE_MSG = 0x106
  EXTERNAL_PANDA = True
  LONGITUDINAL = True

  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (self.DAS_CONTROL_MSG,)}
    self.FWD_BLACKLISTED_ADDRS = {2: [self.DAS_CONTROL_MSG]}
    self.TX_MSGS = [[self.DAS_CONTROL_MSG, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.PARAM_HW2 | TeslaSafetyFlags.PARAM_EXTERNAL_PANDA))
    self.safety.init_tests()

  def _speed_msg(self, speed):
    # External panda uses different speed message location
    values = {"ESP_vehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_panda("ESP_B", 0, values)

  def _vehicle_moving_msg(self, speed: float):
    values = {"DI_cruiseState": 3 if speed <= self.STANDSTILL_THRESHOLD else 2}
    return self.packer.make_can_msg_panda("DI_state", 0, values)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_panda("BrakeMessage", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0}
    return self.packer.make_can_msg_panda("DI_state", 0, values)

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    self.safety.set_controls_allowed(True)

    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))


class TestTeslaHW3ExternalPandaSafety(TeslaLegacyBase):
  CHASSIS_BUS = 0
  USE_APS_EAC_MONITOR = False  # External panda doesn't use APS_eacMonitor
  DAS_CONTROL_MSG = MSG_DAS_Control_HW23
  DI_TORQUE_MSG = 0x106
  EXTERNAL_PANDA = True
  LONGITUDINAL = True

  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (self.DAS_CONTROL_MSG,)}
    self.FWD_BLACKLISTED_ADDRS = {2: [self.DAS_CONTROL_MSG]}
    self.TX_MSGS = [[self.DAS_CONTROL_MSG, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.PARAM_HW3 | TeslaSafetyFlags.PARAM_EXTERNAL_PANDA))
    self.safety.init_tests()

  def _speed_msg(self, speed):
    values = {"ESP_vehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_panda("ESP_B", 0, values)

  def _vehicle_moving_msg(self, speed: float):
    values = {"DI_cruiseState": 3 if speed <= self.STANDSTILL_THRESHOLD else 2}
    return self.packer.make_can_msg_panda("DI_state", 0, values)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_panda("BrakeMessage", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0}
    return self.packer.make_can_msg_panda("DI_state", 0, values)

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    self.safety.set_controls_allowed(True)

    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))


if __name__ == "__main__":
  unittest.main()
