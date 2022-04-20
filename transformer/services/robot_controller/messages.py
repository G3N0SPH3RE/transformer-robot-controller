from enum import Enum
from typing import ClassVar, List, Dict, Union
from dataclasses import dataclass, InitVar, field
from loguru import logger
from services.robot_controller.utils import compute_CRC16
from services.robot_controller.api import ExternalCommandType
import struct

class AxisCommunicationException(BaseException):
    pass

'''
CONSTANTS
'''

class StartCodes(Enum):
    READ_OBJECT = 1 << 7 | 1 << 5 | 1 << 2
    WRITE_OBJECT = 1 << 7 | 1 << 6 | 1 << 2
    RESPONSE = 1 << 2 | 1
    ACK = 1 << 2 | 1 << 1
    NACK = 1 << 2 | 1 << 1 | 1

class FrameIdentifiers(Enum):
    READ_OBJECT = 1 << 3 | 1 << 1
    WRITE_OBJECT = 1 << 3 | 1 << 2
    RESPONSE = 0
    ACK = 0
    NACK = 0

class ResponseStartCodes(Enum):
    RESPONSE = 1 << 2 | 1
    ACK = 1 << 2 | 1 << 1
    NACK = 1 << 2 | 1 << 1 | 1

data_bus_formats = {
    550: {1: 'H'},
    680: {5: 'f'},
    681: {9: 'f', 26: 'f'},
    682: {5: 'I', 6: 'I'},
    688: {2: 'f'},
    1000: {4: 'H'},
    1901: {1: 'f', 2: 'f', 3: 'f', 4: 'f', 5: 'f', 6: 'f', 7: 'f', 8: 'f', 9: 'f'},
    1902: {1: 'f', 2: 'f', 3: 'f', 4: 'f', 5: 'f', 6: 'f', 7: 'f', 8: 'f', 9: 'f'},
    1903: {1: 'H', 2: 'H', 3: 'H', 4: 'H', 5: 'H', 6: 'H', 7: 'H', 8: 'H', 9: 'H'},
    1904: {1: 'H', 2: 'H', 3: 'H', 4: 'H', 5: 'H', 6: 'H', 7: 'H', 8: 'H', 9: 'H'},
    1905: {1: 'H', 2: 'H', 3: 'H', 4: 'H', 5: 'H', 6: 'H', 7: 'H', 8: 'H', 9: 'H'},
    1906: {1: 'I', 2: 'I', 3: 'I', 4: 'I', 5: 'I', 6: 'I', 7: 'I', 8: 'I', 9: 'I'},
    1907: {1: 'I', 2: 'I', 3: 'I', 4: 'I', 5: 'I', 6: 'I', 7: 'I', 8: 'I', 9: 'I'},
    1908: {1: 'I', 2: 'I', 3: 'I', 4: 'I', 5: 'I', 6: 'I', 7: 'I', 8: 'I', 9: 'I'},
    1909: {1: 'I', 2: 'I', 3: 'I', 4: 'I', 5: 'I', 6: 'I', 7: 'I', 8: 'I', 9: 'I'},
}

type_conversions = {
    'f': float,
    'H': int,
    'I': int
}

object_table = {
    #C3 status values
    'unfiltered_acceleration': (682, 5),
    'filtered_acceleration': (682, 6),
    'demanded_acceleration': (682, 4),
    'actual_current': (688, 2),
    'demanded_jerk': (699, 4),
    'actual_position': (680, 5),
    'actual_filtered_velocity': (681, 9),
    'actual_filtered_velocity_mps': (681, 26),
    'current_error_code': (550, 1),

    #IEC status indicators
    'axis_running': (1904, 1),
    'axis_error': (1903, 1),
    'stop_position': (1901, 2),
    'enabled': (1905, 5),
    'status_word_2': (1000, 4),

    #IEC motion commands
    'axis_command': (1905, 1),
    'position_command': (1901, 1),
    'velocity_command': (1902, 1),
    'acceleration_command': (1906, 1),
    'deceleration_command': (1907, 1),
    'jerk_command': (1908, 1),
    'deceleration_jerk_command': (1909, 1),
    'execute_command': (1905, 5),

    #IEC IO Commands
    'io_point_address': (1905, 6),
    'io_point_value': (1905, 7)
}

command_table = {
    'enable_1': 1,
    'enable_2': 2,
    'stop': 9,
    'reset': 10,
    'home_axis': 11,
    'absolute_positioning': 12,
    'relative_positioning': 13,
    'disable': 14,
    'toggle_IO_point': 15
}


'''
INTERNAL MESSAGES
'''

@dataclass
class AxisCommand:
    command_type: ExternalCommandType = None

@dataclass
class AxisMotionCommand():
    target_position: float
    target_velocity: float = None
    target_acceleration: float = None
    target_deceleration: float = None
    target_jerk: float = None
    target_deceleration_jerk: float = None

    #command_type: int = -1

    default_value_map: ClassVar[Dict] = {
        'target_velocity': 'default_velocity',
        'target_acceleration': 'default_acceleration',
        'target_deceleration': 'default_deceleration',
        'target_jerk': 'default_jerk',
        'target_deceleration_jerk': 'default_deceleration_jerk'
    }


@dataclass
class RobotCommand:
    command_type: ExternalCommandType
    emergency: bool = False

    x: AxisCommand = None
    y: AxisCommand = None
    z: AxisCommand = None
    c: AxisCommand = None

    point_address: int = None
    target_axis_label: str = None
    point_function: str = ''
    point_state: bool = False

    cell_component: str = None
    cell_subcomponent: str = None
    state: str = None

    command_id: int = -1

@dataclass
class RobotMotionCommand:
    x: AxisMotionCommand = None
    y: AxisMotionCommand = None
    z: AxisMotionCommand = None
    c: AxisMotionCommand = None

    move_id: ClassVar[int] = -1


'''
SERIAL MESSAGES
'''

### MESSAGE SUPERTYPES ###

@dataclass
class SerialMessage:
    axis_address: int
    object_index: int
    object_subindex: int

    set_value: float
    write_command: bool = False
    received_data: bytes = None
    format: str = 'binary'

    def __post_init__(self):
        if self.received_data != None:
            if self.format == 'ascii':
                if not self.write_command:
                    #Parse returned value from GET command
                    if self.received_data[0] == '!':
                        #error in message
                        a=5
                    else:
                        self.get_value = self.process_message_specific_return_data(self.received_data.decode().strip() if type(self.received_data) == bytes else self.received_data.strip())
            else:
                raise NotImplementedError
        elif self.set_value != None:
            self.write_command = True


    def process_message_specific_return_data(self, data: str):
        pass

    def SerializeToASCII(self):
        return ((str(self.axis_address) if self.axis_address != None else '') + 'O' + '.'.join(
            [str(self.object_index), str(self.object_subindex)]) + ('=' + str(self.set_value) if self.write_command else '') + '\r').encode('ascii')

    def SerializeToBinary(self):
        raise NotImplementedError

    def Serialize(self):
        return (self.SerializeToASCII() if self.format == 'ascii' else self.SerializeToBinary())

@dataclass
class AxisMotionStateMessage:
    axis_address: int = None
    set_value: float = None
    get_value: float = None

    def process_message_specific_return_data(self, data: str):
        try:
            return float(data)
        except Exception as e:
            a=5
            raise ValueError

@dataclass
class AxisStatusFlagMessage:
    axis_address: int = None
    set_value: float = None
    get_value: float = None

    def process_message_specific_return_data(self, data: str) -> bool:
        return int(data) == 1

### MOTION STATUS MESSAGES ####

@dataclass
class CurrentPositionMessage(AxisMotionStateMessage, SerialMessage):
    object_index: int = object_table['actual_position'][0]
    object_subindex: int = object_table['actual_position'][1]

@dataclass
class CurrentVelocityMessage(AxisMotionStateMessage, SerialMessage):
    #axis_address: int = None
    object_index: int = object_table['actual_filtered_velocity_mps'][0]
    object_subindex: int = object_table['actual_filtered_velocity_mps'][1]

@dataclass
class CurrentAccelerationMessage(AxisMotionStateMessage, SerialMessage):
    #axis_address: int = None
    object_index: int = object_table['filtered_acceleration'][0]
    object_subindex: int = object_table['filtered_acceleration'][1]

@dataclass
class CurrentCurrentMessage(AxisMotionStateMessage, SerialMessage):
    #axis_address: int = None
    object_index: int = object_table['actual_current'][0]
    object_subindex: int = object_table['actual_current'][1]

@dataclass
class CurrentErrorMessage(AxisMotionStateMessage, SerialMessage):
    #axis_address: int = None
    object_index: int = object_table['current_error_code'][0]
    object_subindex: int = object_table['current_error_code'][1]

### IEC STATUS MESSAGES ###

@dataclass
class AxisRunningMessage(AxisStatusFlagMessage, SerialMessage):
    object_index: int = object_table['axis_running'][0]
    object_subindex: int = object_table['axis_running'][1]

@dataclass
class AxisErrorMessage(AxisStatusFlagMessage, SerialMessage):
    object_index: int = object_table['axis_error'][0]
    object_subindex: int = object_table['axis_error'][1]

@dataclass
class StopPositionMessage(AxisMotionStateMessage, SerialMessage):
    object_index: int = object_table['stop_position'][0]
    object_subindex: int = object_table['stop_position'][1]

@dataclass
class EnabledMessage(AxisMotionStateMessage, SerialMessage):
    object_index: int = object_table['enabled'][0]
    object_subindex: int = object_table['enabled'][1]

@dataclass
class StatusWord2Message(AxisStatusFlagMessage, SerialMessage):
    object_index: int = object_table['status_word_2'][0]
    object_subindex: int = object_table['status_word_2'][1]

### IEC COMMAND MESSAGES ####

@dataclass
class TargetPositionMessage(SerialMessage):
    set_value: float = None

    object_index: int = object_table['position_command'][0]
    object_subindex: int = object_table['position_command'][1]

@dataclass
class TargetVelocityMessage(SerialMessage):
    set_value: float = None

    object_index: int = object_table['velocity_command'][0]
    object_subindex: int = object_table['velocity_command'][1]

@dataclass
class TargetAccelerationMessage(SerialMessage):
    set_value: float = None

    object_index: int = object_table['acceleration_command'][0]
    object_subindex: int = object_table['acceleration_command'][1]

@dataclass
class TargetDecelerationMessage(SerialMessage):
    set_value: float = None

    object_index: int = object_table['deceleration_command'][0]
    object_subindex: int = object_table['deceleration_command'][1]

@dataclass
class TargetJerkMessage(SerialMessage):
    set_value: float = None

    object_index: int = object_table['jerk_command'][0]
    object_subindex: int = object_table['jerk_command'][1]

@dataclass
class TargetDecelerationJerkMessage(SerialMessage):
    set_value: float = None

    object_index: int = object_table['deceleration_jerk_command'][0]
    object_subindex: int = object_table['deceleration_jerk_command'][1]

@dataclass
class ExecuteMoveCommand(SerialMessage):
    set_value: float = 1

    object_index: int = object_table['execute_command'][0]
    object_subindex: int = object_table['execute_command'][1]

@dataclass
class AcknowledgeCommandError(SerialMessage):
    set_value: float = 0

    object_index: int = object_table['axis_error'][0]
    object_subindex: int = object_table['axis_error'][1]

@dataclass
class EnableAxisBit1Command(SerialMessage):
    set_value: float = command_table['enable_1']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class EnableAxisBit2Command(SerialMessage):
    set_value: float = command_table['enable_2']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class AbsolutePositioningMessage(SerialMessage):
    set_value: float = command_table['absolute_positioning']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class HomingMessage(SerialMessage):
    set_value: float = command_table['home_axis']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class ResetMessage(SerialMessage):
    set_value: float = command_table['reset']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class DisableMessage(SerialMessage):
    set_value: float = command_table['disable']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class StopMessage(SerialMessage):
    set_value: float = command_table['stop']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class IOPointCommandMessage(SerialMessage):
    set_value: float = command_table['toggle_IO_point']

    object_index: int = object_table['axis_command'][0]
    object_subindex: int = object_table['axis_command'][1]

@dataclass
class IOPointAddressMessage(SerialMessage):
    set_value: int = None

    object_index: int = object_table['io_point_address'][0]
    object_subindex: int = object_table['io_point_address'][1]

@dataclass
class IOPointValueMessage(SerialMessage):
    set_value: int = None

    object_index: int = object_table['io_point_value'][0]
    object_subindex: int = object_table['io_point_value'][1]

update_message_sequence = [
    {'message_type': CurrentPositionMessage, 'attribute_name': 'current_position'},
    {'message_type': CurrentVelocityMessage, 'attribute_name': 'current_velocity'},
    {'message_type': CurrentAccelerationMessage, 'attribute_name': 'current_acceleration', 'transform': True},
    {'message_type': CurrentCurrentMessage, 'attribute_name': 'current_current'},
    {'message_type': AxisRunningMessage, 'attribute_name': 'axis_running'},
    {'message_type': AxisErrorMessage, 'attribute_name': 'axis_error'},
    {'message_type': StopPositionMessage, 'attribute_name': 'stop_position'},
    {'message_type': StatusWord2Message, 'attribute_name': 'io_status_bits'},
    {'message_type': CurrentErrorMessage, 'attribute_name': 'current_error_code'},
]

command_message_sequence = [
    {'message_type': TargetPositionMessage, 'attribute_name': 'position_command', 'command_attribute_name': 'target_position'},
    {'message_type': TargetVelocityMessage, 'attribute_name': 'velocity_command', 'command_attribute_name': 'target_velocity'},
    {'message_type': TargetAccelerationMessage, 'attribute_name': 'acceleration_command', 'command_attribute_name': 'target_acceleration'},
    {'message_type': TargetDecelerationMessage, 'attribute_name': 'deceleration_command', 'command_attribute_name': 'target_deceleration'},
    {'message_type': TargetJerkMessage, 'attribute_name': 'jerk_command', 'command_attribute_name': 'target_jerk'},
    {'message_type': TargetDecelerationJerkMessage, 'attribute_name': 'deceleration_jerk_command', 'command_attribute_name': 'target_deceleration_jerk'}
]

reset_message_sequence = [
    {'message_type': ResetMessage}
]

disable_message_sequence = [
    {'message_type': DisableMessage}
]

execute_message_sequence = [
    {'message_type': ExecuteMoveCommand}
]

acknowledge_command_error_message_sequence = [
    {'message_type': AcknowledgeCommandError}
]

enable_message_sequence = [
    {'message_type': EnableAxisBit1Command},
    {'message_type': EnableAxisBit2Command}
]

absolute_position_message_sequence = [
    {'message_type': AbsolutePositioningMessage}
]

homing_message_sequence = [
    {'message_type': TargetPositionMessage, 'attribute_name': 'zero_offset', 'command_attribute_name': 'target_position'},
    {'message_type': HomingMessage}
]

stop_message_sequence = [
    {'message_type': TargetDecelerationMessage, 'attribute_name': 'default_deceleration', 'command_attribute_name': 'target_deceleration'},
    {'message_type': TargetDecelerationJerkMessage, 'attribute_name': 'default_deceleration_jerk', 'command_attribute_name': 'target_deceleration_jerk'},
    {'message_type': StopMessage}
]

io_point_message_sequence = [
    {'message_type': IOPointAddressMessage, 'command_attribute_name': 'io_point_address'},
    {'message_type': IOPointValueMessage, 'command_attribute_name': 'io_point_value'},
    {'message_type': IOPointCommandMessage}
]

@dataclass
class BinaryTelegram:
    #received_data: bytes

    byte_order_string: ClassVar[str] = '>'
    format_string: ClassVar[str]
    payload: ClassVar[str]
    length: ClassVar[int]
    crc16: ClassVar[int]

    def Serialize(self):
        pass

    def create_format_string(self):
        return self.format_string

    def process_response_data(self, response_data: List):
        pass

    def Deserialize(self):
        self.create_format_string()

        try:
            response_data = struct.unpack(self.format_string, self.received_data)

            #check crc
            assert(response_data[-1] == compute_CRC16(self.received_data[:-2]))
            self.process_response_data(response_data)

            self.success = True
        except struct.error as e:
            #Wrong size payload?
            if "unpack requires a buffer of " in e.args[0]:
                # wrong size payload, continue
                pass
            else:
                a=5
        except AssertionError as e:
            logger.warning("CRC check failed for message sequence {}".format(str(self.expected_message_sequence)))
            raise AxisCommunicationException("CRC validation failed")
        except Exception as e:
            a=5

@dataclass
class BinaryStartCode(BinaryTelegram):
    received_data: bytes = None
    received_code: InitVar[int] = None
    frame_id: FrameIdentifiers = 0
    PLC: bool = True
    gateway: bool = False
    address: bool = True

    value: ClassVar[int] = 0

    def __post_init__(self, received_code: int = None):
        if received_code != None:
            self.frame_id = StartCodes(received_code)
        else:
            self.value = self.get_value()

    def get_value(self):
        return self.frame_id.value << 4 | self.PLC << 3 | 1 << 2 | self.gateway << 1 | self.address


@dataclass
class WriteObjectTelegram(BinaryTelegram):
    index: int
    subindex: int
    data: Union[int, float]
    data_format: str = None

    #length: ClassVar[int]
    #crc16: ClassVar[int]

    received_data: bytes = None
    axis_address: int = None

    start_code: ClassVar[BinaryStartCode] = -1
    payload: ClassVar[bytes]

    #start_code: ClassVar[BinaryStartCode] = BinaryStartCode(StartCodes.WRITE_OBJECT)
    frame_id: ClassVar[FrameIdentifiers] = 0

    def __post_init__(self):
        self.start_code = BinaryStartCode(frame_id=FrameIdentifiers.WRITE_OBJECT, address=self.axis_address != None)

        if self.received_data == None:
            self.Serialize()

    def Serialize(self):
        if self.data_format == None:
            self.data_format = data_bus_formats[self.index][self.subindex]

        #self.length = struct.calcsize(self.byte_order_string + 'HB' + self.data_format)
        self.length = struct.calcsize(self.byte_order_string + 'HB' + self.data_format) - 1
        self.format_string = 'B' + ('B' if self.axis_address != None else '') + 'BHB' + self.data_format

        self.payload = struct.pack(self.byte_order_string + self.format_string,
                                   *([self.start_code.get_value()] +
                                     [self.axis_address if self.axis_address != None else []] +
                                     [self.length] +
                                     [self.index, self.subindex] +
                                     [type_conversions[self.data_format](self.data)]))

        #self.crc16 = compute_CRC16(struct.pack('>HB' + self.data_format, self.index, self.subindex, self.data))
        self.crc16 = compute_CRC16(self.payload)
        self.payload += struct.pack('>H', self.crc16)

        return self.payload

@dataclass
class ReadObjectTelegram(BinaryTelegram):
    indices: List[int]
    subindices: List[int]
    axis_address: int = None
    received_data: bytes = None

    length: ClassVar[int]
    telegram: ClassVar[bytes]
    crc16: ClassVar[int]

    start_code: ClassVar[BinaryStartCode] = -1
    format_string: ClassVar[str] = ''

    def __post_init__(self):
        self.start_code = BinaryStartCode(frame_id=FrameIdentifiers.READ_OBJECT, address=self.axis_address != None)
        self.payload = self.Serialize()

    def Serialize(self):
        assert(len(self.indices) == len(self.subindices))
        data_format_string = ''.join(len(self.indices)*['HB'])
        request_addresses = []
        for indices in zip(self.indices, self.subindices):
            request_addresses += list(indices)

        self.length = struct.calcsize(self.byte_order_string + data_format_string)-1
        self.format_string = 'B' + ('' if self.axis_address == None else 'B') + 'B' + data_format_string

        self.telegram = struct.pack(self.byte_order_string + self.format_string,
                                    *([self.start_code.get_value()] +
                                      [self.axis_address if self.axis_address != None else []] +
                                      [self.length] +
                                      request_addresses))

        self.crc16 = compute_CRC16(self.telegram)

        return self.telegram + struct.pack('>H', self.crc16)

@dataclass
class ReadObjectResponseTelegram(BinaryTelegram):
    received_data: bytes
    expected_message_sequence: List[Dict]

    #format_string: ClassVar[str] = ''
    start_code: ClassVar[BinaryStartCode] = ResponseStartCodes.RESPONSE
    success: ClassVar[bool] = False

    def __post_init__(self):#, received_data: bytes, expected_message_sequence: List[Dict]):
        assert(self.start_code.value == self.received_data[0])
        self.Deserialize()

    def create_format_string(self):
        self.format_string = self.byte_order_string + 'BB'
        for message in self.expected_message_sequence:
            self.format_string += data_bus_formats[message['message_type'].object_index][
                message['message_type'].object_subindex]
        # crc bytes
        self.format_string += 'H'

    def process_response_data(self, response_data: List):
        try:
            rx_data_values = response_data[2:-1]
            for message, rx_value in zip(self.expected_message_sequence, rx_data_values):
                if 'attribute_name' in message.keys():
                    setattr(self, message['attribute_name'], rx_value)
        except Exception as e:
            a=5

@dataclass
class AckTelegram(BinaryTelegram):
    received_data: bytes

    start_code: ClassVar[BinaryStartCode] = ResponseStartCodes.ACK
    format_string: ClassVar[str] = 'BBHH'
    success: ClassVar[bool] = False

    def __post_init__(self):
        self.format_string = self.byte_order_string + self.format_string
        self.Deserialize()

@dataclass
class MessageProcessed:
    received: bool = False
    correct: bool = False
    response: str = ''
    received_telegram: BinaryTelegram = None