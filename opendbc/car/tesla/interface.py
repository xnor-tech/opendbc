from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, CAR, TeslaFlags, LEGACY_CARS
from opendbc.car.tesla.radar_interface import RadarInterface


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if candidate in LEGACY_CARS:
      PARAM_POWERTRAIN = 2
      PARAM_HW2 = 4

      if not any(0x201 in f for f in fingerprint.values()):
        ret.flags |= TeslaFlags.NO_SDM1.value

      ret.safetyConfigs = [
        get_safety_config(99, 0),
        get_safety_config(99, PARAM_POWERTRAIN),
      ]
      if candidate == CAR.TESLA_MODEL_S_HW2:
        ret.safetyConfigs[0].safetyParam |= PARAM_HW2
        ret.safetyConfigs[1].safetyParam |= PARAM_HW2
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = not candidate == CAR.TESLA_MODEL_S_RAVEN

    ret.alphaLongitudinalAvailable = True
    if alpha_long or candidate in LEGACY_CARS:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret
