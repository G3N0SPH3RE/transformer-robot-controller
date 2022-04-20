import socket, struct
from typing import ClassVar, Dict
from dataclasses import dataclass, field, fields, InitVar
import serial

client = True
plc_ip = "192.168.1.60"
plc_port = 6969
#BYTE_ORDER = 'big'

@dataclass
class BaseMessage:
    _bytestring: bytes
    #format: field(default_factory=dict)
    format: field(default_factory=dict)
    byte_order: ClassVar[str] = 'big'

    def __post_init__(self):
        data = struct.unpack(('>' if self.byte_order == 'big' else '') + ''.join([v for k, v in self.format.items()]),
                             self._bytestring)
        data_ndx = 0
        for field in fields(self):
            if field.name[0] == '_':
                continue
            setattr(self, field.name, data[data_ndx])
            data_ndx += 1

        # for fieldname in self.fields():
        #     setattr(self, fieldname, data[data_ndx])
        #     data_ndx += 1
        a = 5

@dataclass
class AxisStateMessage(BaseMessage):
    position: float = 0
    velocity: float = 0
    acceleration: float = 0
    jerk: float = 0
    current: float = 0
    homed: int = 0
    energized: int = 0
    in_progress: int = 0

    format: ClassVar[Dict[str, str]] = {'position': 'f', 'velocity': 'f', 'acceleration': 'f', 'jerk': 'f',
              'current': 'f', 'homed': 'B', 'energized': 'B', 'in_progress': 'B'}

    # def __post_init__(self, byte_order: str, bytestring: bytes):
    #     data = struct.unpack('!' if self.byte_order == 'big' else '' + \
    #                   [v for k, v in self.format].join(''), \
    #                   bytestring)

        # data_ndx = 0
        # for fieldname in self.fields():
        #     setattr(self, fieldname, data[data_ndx])
        #     data_ndx += 1
        # a = 5

    # def __init__(self):
    #     self.format = {'position': 'f', 'velocity': 'f', 'acceleration': 'f', 'jerk': 'f', 'current': 'f', 'homed': 'B',
    #                    'energized': 'B', 'in_progress': 'B'}


def open_socket():
    plc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if not client:
        #plc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        plc_socket.bind((socket.gethostname(), 6969))
        plc_socket.listen()
        print("Listening for connections")
        (clientsocket, address) = plc_socket.accept()
    else:
        plc_socket.connect((plc_ip, plc_port))


    print("Socket connected")
    return plc_socket

if __name__ == "__main__":
    plc_socket = open_socket()

    while True:
        chunk = plc_socket.recv(2048)
        print(chunk)
        a = AxisStateMessage(chunk)


