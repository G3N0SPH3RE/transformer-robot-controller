from typing import ClassVar, Dict
from pydantic import BaseModel, PrivateAttr, validator, root_validator#, dataclasses
from enum import Enum

class ExternalCommandType(Enum):
    ENABLE = 1
    DISABLE = 2
    STOP = 3
    HOME = 4
    RESET = 5
    ABORT = 6
    SET_IO_POINT = 7
    SET_CELL_SUBCOMPONENT_STATE = 8

### HTTP Requests ###
class HTTPAxisPositioningCommand(BaseModel):
    target_position: float
    target_velocity: float = None
    target_acceleration: float = None
    target_deceleration: float = None
    target_jerk: float = None
    target_deceleration_jerk: float = None

    default_value_map: ClassVar[Dict] = {
        'target_velocity': 'default_velocity',
        'target_acceleration': 'default_acceleration',
        'target_deceleration': 'default_deceleration',
        'target_jerk': 'default_jerk',
        'target_deceleration_jerk': 'default_deceleration_jerk'
    }

class HTTPRobotPositioningCommand(BaseModel):
    x: HTTPAxisPositioningCommand = None
    y: HTTPAxisPositioningCommand = None
    z: HTTPAxisPositioningCommand = None
    c: HTTPAxisPositioningCommand = None

    _command_id: int = PrivateAttr(-1)

    def set_command_id(self, id: int):
        self._command_id = id

    def get_command_id(self) -> int:
        return self._command_id

class HTTPRobotCommand(BaseModel):
    command_type: ExternalCommandType
    axis_label: str = None

    _command_id: int = PrivateAttr(-1)

    def set_command_id(self, id: int):
        self._command_id = id

    def get_command_id(self) -> int:
        return self._command_id

class HTTPAxisCommand(BaseModel):
    command_arg: int = None

    target_position: float = None
    target_velocity: float = None
    target_acceleration: float = None
    target_deceleration: float = None
    target_jerk: float = None
    target_deceleration_jerk: float = None

class HTTPCommand(BaseModel):
    command_arg: int = None
    state: str = ''
    sequence_type: str = ''

    origin: str = ''
    destination: str = ''

    point_state: int = None

    gripper_state: str = None

    subcomponent_state: str = None

    x: HTTPAxisCommand = None
    y: HTTPAxisCommand = None
    z: HTTPAxisCommand = None
    c: HTTPAxisCommand = None

# class HTTPCommandError(BaseModel):
#     error_string: str

class HTTPResponse(BaseModel):
    error: bool = False
    error_string: str = ''
    success_string: str = ''
    data: Dict = {}

    @validator('error', always=True)
    def validate_args(cls, values):
        return values

    @root_validator(pre=False)
    def validate_model(cls, values):
        if values['error_string'] != '':
            values['error'] = True
        else:
            values['error'] = False

        if values['success_string'] != '':
            values['error'] = False

        return values

class DataHTTPResponse(BaseModel):
    data: Dict
    error: bool = False
    response: str = ''

class BooleanHTTPResponse(BaseModel):
    error: bool = False
    response: str = ''
    # error_string: str = ''
    # success_string: str = ''
    #
    # @validator('error', always=True)
    # def validate_args(cls, values):
    #     return values
    #
    # @root_validator(pre=False)
    # def validate_model(cls, values):
    #     if values['error_string'] != '':
    #         values['error'] = True
    #     else:
    #         values['error'] = False
    #
    #     if values['success_string'] != '':
    #         values['error'] = False
    #
    #     return values