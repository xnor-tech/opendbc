"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from enum import StrEnum, IntEnum

from opendbc.car import Bus, create_button_events, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.rivian.values import DBC
from opendbc.sunnypilot.car.rivian.values import RivianFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type


class VDMUserAdasRequest(IntEnum):
  IDLE = 0
  UP_1 = 1
  UP_2 = 2
  DOWN_1 = 3
  DOWN_2 = 4


VDM_BUTTON_MAP = {
  VDMUserAdasRequest.DOWN_2: ButtonType.lkas,    # Toggle MADS
}


MAX_SET_SPEED = 85 * CV.MPH_TO_MS
MIN_SET_SPEED = 20 * CV.MPH_TO_MS


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.set_speed = 10
    self.increase_button = False
    self.decrease_button = False
    self.distance_button = 0
    self.increase_counter = 0
    self.decrease_counter = 0
    self.vdm_user_adas_request = 0

    # Button hold timers for time-based actions
    self.down_button_hold_counter = 0
    self.mads_toggle_hold_processed = False

  def update_stalk_controls(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> list:
      cp = can_parsers[Bus.pt]
      self.vdm_user_adas_request = cp.vl["VDM_AdasSts"]["VDM_UserAdasRequest"]

      button_events = []

      if self.vdm_user_adas_request == VDMUserAdasRequest.DOWN_2:
          # Button is HELD
          self.down_button_hold_counter += 1

          # 2s+: Toggle MADS
          if self.down_button_hold_counter >= 200 and not self.mads_toggle_hold_processed:
              button_events.extend([structs.CarState.ButtonEvent(pressed=True, type=ButtonType.lkas),
                                    structs.CarState.ButtonEvent(pressed=False, type=ButtonType.lkas)])
              self.mads_toggle_hold_processed = True

      else:
          # Button is RELEASED
          if self.down_button_hold_counter > 0:
              # 0.3s-1s: Set cruise speed to current cluster speed (triggers on release)
              if 30 <= self.down_button_hold_counter < 100 and self.CP.openpilotLongitudinalControl:
                  self.set_speed = max(MIN_SET_SPEED, min(self.set_speed, MAX_SET_SPEED))
                  ret.cruiseState.speed = self.set_speed

              # Reset counter and flag
              self.down_button_hold_counter = 0
              self.mads_toggle_hold_processed = False

      return button_events

  def update_longitudinal_upgrade(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> list:
    cp_park = can_parsers[Bus.alt]
    cp_adas = can_parsers[Bus.adas]

    button_events = []
    prev_increase_button = self.increase_button
    prev_decrease_button = self.decrease_button

    if self.CP.openpilotLongitudinalControl:
      # distance scroll wheel
      right_scroll = cp_park.vl["WheelButtons"]["RightButton_Scroll"]
      if right_scroll != 255:
        if self.distance_button != right_scroll:
          button_events.extend([structs.CarState.ButtonEvent(pressed=False, type=ButtonType.gapAdjustCruise)])
        self.distance_button = right_scroll

      # button logic for set-speed
      self.increase_button = cp_park.vl["WheelButtons"]["RightButton_RightClick"] == 2
      self.decrease_button = cp_park.vl["WheelButtons"]["RightButton_LeftClick"] == 2

      self.increase_counter = self.increase_counter + 1 if self.increase_button else 0
      self.decrease_counter = self.decrease_counter + 1 if self.decrease_button else 0

      metric = cp_adas.vl["Cluster"]["Cluster_Unit"] == 0
      conversion = CV.KPH_TO_MS if metric else CV.MPH_TO_MS
      long_press_step = 10.0 if metric else 5.0
      set_speed_converted = self.set_speed * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)

      if self.increase_button:
        if self.increase_counter % 66 == 0:
          self.set_speed = (int(math.ceil((set_speed_converted + 1) / long_press_step)) * long_press_step) * conversion
        elif not prev_increase_button:
          self.set_speed += conversion

      if self.decrease_button:
        if self.decrease_counter % 66 == 0:
          self.set_speed = (int(math.floor((set_speed_converted - 1) / long_press_step)) * long_press_step) * conversion
        elif not prev_decrease_button:
          self.set_speed -= conversion

      if not ret.cruiseState.enabled:
        self.set_speed = ret.vEgoCluster

      self.set_speed = max(MIN_SET_SPEED, min(self.set_speed, MAX_SET_SPEED))
      ret.cruiseState.speed = self.set_speed

    if self.CP.enableBsm:
      ret.leftBlindspot = cp_park.vl["BSM_BlindSpotIndicator"]["BSM_BlindSpotIndicator_Left"] != 0
      ret.rightBlindspot = cp_park.vl["BSM_BlindSpotIndicator"]["BSM_BlindSpotIndicator_Right"] != 0

    return button_events

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    button_events = []

    if self.CP_SP.flags & RivianFlagsSP.LONGITUDINAL_HARNESS_UPGRADE:
      button_events.extend(self.update_longitudinal_upgrade(ret, can_parsers))

    button_events.extend(self.update_stalk_controls(ret, can_parsers))
    ret.buttonEvents = button_events

  @staticmethod
  def get_parser(CP, CP_SP) -> dict[StrEnum, CANParser]:
    messages = {}

    if CP_SP.flags & RivianFlagsSP.LONGITUDINAL_HARNESS_UPGRADE:
      messages[Bus.alt] = CANParser(DBC[CP.carFingerprint][Bus.alt], [], 5)

    return messages
