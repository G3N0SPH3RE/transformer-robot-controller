import serial, yaml, time, asyncio, socket, json, os, websockets
from threading import Thread, Lock
from queue import Queue
from typing import Dict, Any, List, Union, Tuple
from dataclasses import dataclass, fields
from loguru import logger
from datetime import datetime
from enum import Enum
from transitions import Machine, State
from shutil import copyfile

from . import *
from .api import ExternalCommandType
from .messages import AxisMotionCommand, AxisCommand, RobotMotionCommand, RobotCommand
from .hardware_interface import Compax3M
from .utils import MachineConfigurationException, MotionControllerException
from .machine_interface import MachineTool

class MotionControllerStates(Enum):
    ERROR = -1
    MANUAL = 0
    STOPPED = 1
    SEQUENCING = 2
    SEQUENCING_TO_PICK = 3
    SEQUENCING_TO_PLACE = 4

class SequenceStates(Enum):
    ERROR = -1
    BEGIN = 0
    BEGIN_SAFE_POSITION = 1
    END_SAFE_POSITION = 2
    TRAVERSE = 3
    #POSITION_PALLET = 4
    POSITION_PALLET_FOR_PICK = 4
    POSITION_PALLET_FOR_PLACE = 5
    INSERT_PALLET = 6
    RAISE_PALLET = 7
    LOWER_PALLET = 8
    WITHDRAW_PALLET = 9
    PLACE_PALLET = 10
    PICK_PALLET = 11
    WAITING_FOR_MACHINE_ACK = 12
    PICK_LIFT = 13
    DONE = 14

class SequenceTypes(Enum):
    PICK = 0
    PLACE = 1


SEQUENCE_STATES = [
    State(name=SequenceStates.BEGIN),
    #State(name=SequenceStates.BEGIN_SAFE_POSITION, on_enter=['update_sequence_state_history', 'set_safe_position_target']),
    State(name=SequenceStates.BEGIN_SAFE_POSITION, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.END_SAFE_POSITION, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.TRAVERSE, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.POSITION_PALLET_FOR_PICK, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.POSITION_PALLET_FOR_PLACE, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.INSERT_PALLET, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.LOWER_PALLET, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.RAISE_PALLET, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.WITHDRAW_PALLET, on_enter=['update_sequence_state_history', 'set_motion_position_target']),
    State(name=SequenceStates.PLACE_PALLET, on_enter=['update_sequence_state_history', 'place_pallet']),
    State(name=SequenceStates.PICK_PALLET, on_enter=['update_sequence_state_history', 'pick_pallet']),
    State(name=SequenceStates.DONE, on_enter=['update_sequence_state_history']),
    State(name=SequenceStates.ERROR, on_enter=['update_sequence_state_history', 'enter_sequence_error_mode'])
]

SEQUENCE_WAYPOINTS = {
    #SequenceStates.BEGIN: {"reach": None, "traverse": None, "vertical": None, "swing": None},
    SequenceStates.BEGIN_SAFE_POSITION: {"reach": "safe_position", "traverse": None, "vertical": None, "swing": None},
    SequenceStates.END_SAFE_POSITION: {"reach": "safe_position", "traverse": None, "vertical": None, "swing": None},
    SequenceStates.TRAVERSE: {"reach": None, "traverse": "traverse_end", "vertical": None, "swing": None},
    #SequenceStates.POSITION_PALLET: {"reach": None, "traverse": None, "vertical": "clearance_height", "swing": "swing_end"},
    SequenceStates.POSITION_PALLET_FOR_PICK: {"reach": None, "traverse": None, "vertical": "vertical_end", "swing": "swing_end"},
    SequenceStates.POSITION_PALLET_FOR_PLACE: {"reach": None, "traverse": None, "vertical": "clearance_height", "swing": "swing_end"},
    SequenceStates.INSERT_PALLET: {"reach": "reach_end", "traverse": None, "vertical": None, "swing": None},
    SequenceStates.LOWER_PALLET: {"reach": None, "traverse": None, "vertical": "vertical_end", "swing": None},
    SequenceStates.RAISE_PALLET: {"reach": None, "traverse": None, "vertical": "clearance_height", "swing": None},
    SequenceStates.WITHDRAW_PALLET: {"reach": "safe_position", "traverse": None, "vertical": None, "swing": None},
    SequenceStates.PLACE_PALLET: {"reach": None, "traverse": None, "vertical": None, "swing": None},
    SequenceStates.PICK_PALLET: {"reach": None, "traverse": None, "vertical": None, "swing": None}
}

SEQUENCE_PLACE_TRANSITIONS = [
    {'trigger': 'next_sequence_step', 'source': SequenceStates.BEGIN, 'dest': SequenceStates.BEGIN_SAFE_POSITION},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.BEGIN_SAFE_POSITION, 'dest': SequenceStates.TRAVERSE, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.TRAVERSE, 'dest': SequenceStates.POSITION_PALLET_FOR_PLACE, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    #{'trigger': 'next_sequence_step', 'source': SequenceStates.POSITION_PALLET, 'dest': SequenceStates.INSERT_PALLET, 'conditions': 'check_motion_position_target_reached', 'before': 'prepare_to_insert_pallet', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.POSITION_PALLET_FOR_PLACE, 'dest': SequenceStates.INSERT_PALLET, 'conditions': ['check_motion_position_target_reached', 'check_machine_ready_for_insertion'], 'before': 'prepare_to_place_pallet', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.INSERT_PALLET, 'dest': SequenceStates.LOWER_PALLET, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.LOWER_PALLET, 'dest': SequenceStates.PLACE_PALLET, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    #{'trigger': 'next_sequence_step', 'source': SequenceStates.INSERT_PALLET, 'dest': SequenceStates.PLACE_PALLET, 'conditions': 'reach_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.PLACE_PALLET, 'dest': SequenceStates.WITHDRAW_PALLET, 'conditions': 'pallet_placed', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.WITHDRAW_PALLET, 'dest': SequenceStates.END_SAFE_POSITION, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.END_SAFE_POSITION, 'dest': SequenceStates.DONE, 'conditions': 'check_motion_position_target_reached', 'before': 'ready_machine_for_operation', 'after': 'remove_last_transition'},
    {'trigger': 'sequencing_to_abort', 'source': '*', 'dest': SequenceStates.ERROR},
]

SEQUENCE_PICK_TRANSITIONS = [
    {'trigger': 'next_sequence_step', 'source': SequenceStates.BEGIN, 'dest': SequenceStates.BEGIN_SAFE_POSITION, 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.BEGIN_SAFE_POSITION, 'dest': SequenceStates.TRAVERSE, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.TRAVERSE, 'dest': SequenceStates.POSITION_PALLET_FOR_PICK, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.POSITION_PALLET_FOR_PICK, 'dest': SequenceStates.INSERT_PALLET, 'conditions': ['check_motion_position_target_reached', 'check_machine_ready_for_insertion'], 'before': 'prepare_to_pick_pallet', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.INSERT_PALLET, 'dest': SequenceStates.PICK_PALLET, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.PICK_PALLET, 'dest': SequenceStates.RAISE_PALLET, 'conditions': 'pallet_picked', 'before': 'prepare_to_withdraw_pallet', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.RAISE_PALLET, 'dest': SequenceStates.WITHDRAW_PALLET, 'conditions': 'check_motion_position_target_reached', 'before': 'prepare_to_withdraw_pallet', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.WITHDRAW_PALLET, 'dest': SequenceStates.END_SAFE_POSITION, 'conditions': 'check_motion_position_target_reached', 'after': 'remove_last_transition'},
    {'trigger': 'next_sequence_step', 'source': SequenceStates.END_SAFE_POSITION, 'dest': SequenceStates.DONE, 'conditions': 'check_motion_position_target_reached', 'before': 'ready_machine_for_operation', 'after': 'remove_last_transition'},
    {'trigger': 'sequencing_to_abort', 'source': '*', 'dest': SequenceStates.ERROR},
]

MACHINE_CALLBACKS = [
    'set_motion_position_target',
    'set_safe_position_target',
    'set_traverse_position_target',
    'set_pallet_position_target',
    'set_reach_position_target',
    'set_withdrawal_position_target',
    'set_vertical_lift_position_target',
    'set_vertical_lower_position_target',
    'place_pallet',
    'pick_pallet',
    'pallet_raised',
    'pallet_lowered',
    'enter_sequence_error_mode',
    'prepare_to_insert_pallet',
    'prepare_to_withdraw_pallet',
    'prepare_to_pick_pallet',
    'prepare_to_place_pallet',
    'ready_machine_for_operation',
    'update_sequence_state_history',
    'remove_last_transition',
    'check_motion_position_target_reached',
    'check_machine_ready_for_insertion',
    'safe_position_target_reached',
    'traverse_target_reached',
    'pallet_position_target_reached',
    'reach_target_reached',
    'pallet_placed',
    'pallet_picked',
    'pallet_withdrawn'
]

@dataclass
class MotionControllerCommandResponse:
    response_string: str
    success: bool

@dataclass
class MachineConfiguration:
    port_address: str = 'COM9'
    baudrate: int = 115200
    communication_type: str = 'rs232'
    communication_timeout: float = 0.1

@dataclass
class MachineSequence:
    sequence_type: SequenceTypes = None
    reach_end: float = None
    traverse_end: float = None
    vertical_end: float = None
    swing_end: float = None

    target_location: str = ''
    clearance_height: float = None

    sequence_id: int = -1

class MotionController(Machine, Thread):
    def __init__(self, configuration_file_name: str, calibration_table_name: str, motion_data_stream_address: Tuple[str, int]):
        # Motion controller states
        states = [
            State(name=MotionControllerStates.MANUAL, on_enter='enter_manual_mode'),
            State(name=MotionControllerStates.STOPPED, on_enter='enter_stop_mode'),
            State(name=MotionControllerStates.SEQUENCING, on_enter='enter_sequence_mode'),
            State(name=MotionControllerStates.SEQUENCING_TO_PICK),
            State(name=MotionControllerStates.SEQUENCING_TO_PLACE),
            State(name=MotionControllerStates.ERROR, on_enter='enter_error_mode')
        ]

        super(MotionController, self).__init__(model=self, states=states, initial=MotionControllerStates.STOPPED)

        self.add_transition(trigger='to_stop_mode', source='*', dest=MotionControllerStates.STOPPED)
        self.add_transition(trigger='to_manual_mode', source=MotionControllerStates.STOPPED,
                            dest=MotionControllerStates.MANUAL, conditions='check_manual_transition')
        self.add_transition(trigger='to_sequence_mode', source=MotionControllerStates.STOPPED,
                            dest=MotionControllerStates.SEQUENCING)
        self.add_transition(trigger='to_error_mode', source='*', dest=MotionControllerStates.ERROR)
        self.add_transition(trigger='to_manual_mode', source=MotionControllerStates.ERROR,
                            dest=MotionControllerStates.MANUAL)

        self.sequence_state_history = []
        self.run_state_method = None

        self.run_flag: bool = True
        self.data_lock = Lock()
        #self.motion_queue: Queue[RobotMotionCommand] = Queue()
        self.motion_queue: Queue[RobotMotionCommand] = Queue()
        self.machine_command_queue: Queue[RobotCommand] = Queue()
        self.sequence_queue: Queue[MachineSequence] = Queue()

        self.machine_setup_configuration: Dict[str, Any] = {}
        self.communication_configuration: Dict[str, Any] = {}
        self.io_point_configuration: Dict[str, Dict[str, int]] = {'output': {}, 'input': {}}
        self.axes_installed: List = []
        self.simulate_axes: bool = False
        self.simulate_io: bool = False
        self.axes: Dict[str, Compax3M] = {}
        self.axis_function_map: Dict[str, str] = {}

        self.port: serial.Serial = serial.Serial()
        self.motion_command_id = 0
        self.machine_command_id = 0
        self.sequence_id = 0

        self.current_machine_sequence: MachineSequence = None
        #self.current_sequence_waypoints: Dict[Dict[str, float]] = None
        self.current_sequence_waypoints: Dict[Dict[str, float]] = None

        self.calibration_table_name: str = calibration_table_name
        self.calibration_defaults: Dict = {}
        self.rack_locations: Dict = {}
        self.machine_locations: Dict = {}
        self.cell_components: Dict = {}
        #self.cell_com

        self.read_machine_configuration(configuration_file_name=configuration_file_name)#, machine_specific_configuration_file_name=machine_specific_configuration_file_name)
        self.read_calibration_table(self.calibration_table_name)
        self.initialize_port()

        self.motion_data_stream_address = motion_data_stream_address
        self.motion_data_publish_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.motion_data_publish_websocket = websockets.serve(self.handle_websocket, host='0.0.0.0', port=8765)
        
        # async with websockets.serve(self.handle_websocket, host='localhost', port=8765):
        #     await asyncio.Future()


        self.per_axis_log_format: List = ['current_position', 'current_velocity', 'current_acceleration', 'current_jerk', 'current_current']

        logger.info("Started motion controller")

        self.motion_logger = logger.bind(motion=True)
        self.motion_logger.trace(self.create_logger_header())

        #Initialize the SM into default state
        self.internal_stop_mode_change()


    '''
    '''
    ### UTILITIES
    def create_logger_header(self):
        log_format = []
        for axis_label in self.axes_installed:
            log_format += (['_'.join([axis_label, column_name]) for column_name in self.per_axis_log_format])

        return ','.join(['time'] + log_format)

    def log_axis_motion(self):
        log_data = []
        log_format = []
        for axis_label in self.axes_installed:
            log_format += (['_'.join([axis_label, column_name]) for column_name in self.per_axis_log_format])
            log_data += ([str(getattr(self.axes[axis_label], col)) for col in self.per_axis_log_format])

        time_string = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        self.motion_logger.trace(",".join([time_string] + log_data))

        axis_data = {key: float(value) for key, value in zip(log_format, log_data)}
        json_message = {
            "timestamp": time_string,
            "axis_data": axis_data
        }

        self.motion_data_publish_socket.sendto(json.dumps(json_message).encode(), self.motion_data_stream_address)

    async def handle_websocket(self, connection: websockets.WebSocketServerProtocol, path):
        a=5
        await connection.recv()
        async for message in connection:
            await connection.send(message)

    def check_motion_stopped(self) -> bool:
        return all([not axis_obj.axis_running for _, axis_obj in self.axes.items()])

    def stop_axes(self):
        for axis_label, axis_obj in self.axes.items():
            command = RobotCommand(command_type=ExternalCommandType.STOP, emergency=True)
            setattr(command, axis_label, AxisCommand(command_type=ExternalCommandType.STOP))
            self.enqueue_machine_command(command)


    async def wait_for_motion_stop(self, timeout: int = None, poll_period: int = None):
        timeout = self.machine_setup_configuration['state_transition_timeout'] if timeout is None else timeout
        poll_period = self.machine_setup_configuration['state_transition_poll_period'] if poll_period is None else poll_period
        wait_start_time = time.time()
        while not self.check_motion_stopped():
            if (time.time() - wait_start_time) > timeout:
                return False
            else:
                await asyncio.sleep(poll_period)
                #time.sleep(poll_period)

        return True

    def lookup_axis_function(self, axis_label: str):
        try:
            return next(key for key, value in self.axis_function_map.items() if value == axis_label)
        except StopIteration:
            raise KeyError

    def initialize_port(self):
        with self.port as port:
            port.baudrate = self.communication_configuration['baudrate']
            port.port = self.communication_configuration['port_address']
            port.bytesize = 8
            port.parity = 'N'
            port.stopbits = 1
            port.timeout = self.communication_configuration['communication_timeout']
            port.xonxoff = 0
            port.rtscts = 0

        try:
            self.port.open()
        except Exception as e:
            print("Serial port open error: " + str(e))

    def read_machine_configuration(self, configuration_file_name: str)-> Dict:
        # Read machine specific configuration
        with open(os.path.join(ROOT_PATH, CONFIG_FILE_DIRECTORY, MACHINE_SPECIFIC_FILE_PREFIX + configuration_file_name), 'r') as config_file:
            config = yaml.load(config_file, Loader=yaml.Loader)
            for param_name, param_value in config['communication'].items():
                if type(param_value) != dict:
                    self.communication_configuration[param_name] = param_value

            for param_name, param_value in config['axes'].items():
                if type(param_value) == list:
                    setattr(self, param_name, param_value)
                else:
                    setattr(self, param_name, param_value)

            for param_name, param_value in config['io'].items():
                setattr(self, param_name, param_value)


        #Read general configuration
        with open(os.path.join(ROOT_PATH, CONFIG_FILE_DIRECTORY, configuration_file_name), 'r') as config_file:
            config = yaml.load(config_file, Loader=yaml.Loader)
            for param_name, param_value in config['machine_setup'].items():
                self.machine_setup_configuration[param_name] = param_value

            for param_name, param_value in config['communication'].items():
                if type(param_value) != dict:
                    self.communication_configuration[param_name] = param_value

            for _, axis_params in config['axes'].items():
                if axis_params['axis_label'] in self.axis_function_map.values():
                    raise MachineConfigurationException("Cannot have multiple axes with the same label")
                if axis_params['axis_function'] in self.axis_function_map.keys():
                    raise MachineConfigurationException("Cannot have multiple axes with the same function")


                if axis_params['axis_label'] in self.axes_installed:
                    self.axis_function_map[axis_params['axis_function']] = axis_params['axis_label']
                    self.axes[axis_params['axis_label']] = Compax3M(self.communication_configuration, **axis_params)

            #Process output points first
            if 'io' in config.keys():
                if 'output' in config['io'].keys():
                    for io_point_function, io_point_params in config['io']['output'].items():
                        self.io_point_configuration['output'][io_point_function] = io_point_params
                        if 'bound_axis_label' in io_point_params.keys() and io_point_params['bound_axis_label'] == '*':
                            for _, axis in self.axes.items():
                                for output_name, output in axis.outputs.items():
                                    if output['point_address'] == io_point_params['point_address']:
                                        axis.outputs.pop(output_name)
                                        break
                                axis.outputs[io_point_function] = {'point_address': io_point_params['point_address'], 'point_state': False}
                        else:
                            if 'bound_axis_label' in io_point_params.keys() and io_point_params['bound_axis_label'] in self.axes_installed:
                                for output_name, output in self.axes[io_point_params['bound_axis_label']].outputs.items():
                                    if output['point_address'] == io_point_params['point_address']:
                                        self.axes[io_point_params['bound_axis_label']].outputs.pop(output_name)
                                        break
                                self.axes[io_point_params['bound_axis_label']].outputs[io_point_function] = {'point_address': io_point_params['point_address'], 'point_state': False}

                if 'input' in config['io'].keys():
                    for io_point_function, io_point_params in config['io']['input'].items():
                        self.io_point_configuration['input'][io_point_function] = io_point_params
                        if 'bound_axis_label' in io_point_params.keys() and io_point_params['bound_axis_label'] == '*':
                            for _, axis in self.axes.items():
                                for input_name, input in axis.inputs.items():
                                    if input['point_address'] == io_point_params['point_address']:
                                        axis.inputs.pop(input_name)
                                        break
                                axis.inputs[io_point_function] = {'point_address': io_point_params['point_address'], 'point_state': False}
                        else:
                            if 'bound_axis_label' in io_point_params.keys() and io_point_params['bound_axis_label'] in self.axes_installed:
                                for input_name, input in self.axes[io_point_params['bound_axis_label']].inputs.items():
                                    if input['point_address'] == io_point_params['point_address']:
                                        self.axes[io_point_params['bound_axis_label']].inputs.pop(input_name)
                                        break
                                self.axes[io_point_params['bound_axis_label']].inputs[io_point_function] = {'point_address': io_point_params['point_address'], 'point_state': False}
                        #self.io_point_configuration['input'][io_point_function] = io_point_params



    def read_calibration_table(self, calibration_table_name: str):
        if not os.path.isfile(os.path.join(ROOT_PATH, CONFIG_FILE_DIRECTORY, MACHINE_SPECIFIC_FILE_PREFIX + calibration_table_name)):
            copyfile(os.path.join(ROOT_PATH, CONFIG_FILE_DIRECTORY, TEMPLATE_FILE_PREFIX + calibration_table_name),
            os.path.join(ROOT_PATH, CONFIG_FILE_DIRECTORY, MACHINE_SPECIFIC_FILE_PREFIX + calibration_table_name))
        with open(os.path.join(ROOT_PATH, CONFIG_FILE_DIRECTORY, MACHINE_SPECIFIC_FILE_PREFIX + calibration_table_name), 'r') as config_file:
            table = yaml.load(config_file, Loader=yaml.Loader)
            for rack_id, rack_location in table['racks'].items():
                self.rack_locations[rack_id] = rack_location

            for machine_id, machine_parameters in table['machines'].items():
                # if 'configuration' in machine_parameters.keys():
                #     self.cell_component_configuration[machine_id] = machine_parameters['configuration']    
                self.machine_locations[machine_id] = machine_parameters['position_calibration']
                self.cell_components[machine_id] = MachineTool(name=machine_id, cell_controller=self, configuration=machine_parameters['configuration'], **machine_parameters['position_calibration'])

            if 'defaults' in table.keys():
                self.calibration_defaults = table['defaults']

    '''
    '''
    ### IO HANDLING
    def set_pallet_clamp(self, clamped: bool) -> bool:
        if "gripper_open" in self.io_point_configuration['output'].keys() and "gripper_close" in self.io_point_configuration['output'].keys() and not self.simulate_io:
            bound_axis_label = self.io_point_configuration['output']['gripper_open']['bound_axis_label']
            point_address = self.io_point_configuration['output']['gripper_open']['point_address']
            self.axes[bound_axis_label].set_IO_point(port=self.port,
                                                     io_point_address=point_address,
                                                     io_point_value=not clamped)

            bound_axis_label = self.io_point_configuration['output']['gripper_close']['bound_axis_label']
            point_address = self.io_point_configuration['output']['gripper_close']['point_address']
            self.axes[bound_axis_label].set_IO_point(port=self.port,
                                                     io_point_address=point_address,
                                                     io_point_value=clamped)
            return True
        return False

    def set_table_clamp(self, clamped: bool) -> bool:
        target_location = self.current_machine_sequence.target_location
        if target_location in self.cell_components.keys() and type(self.cell_components[target_location]) == MachineTool and self.cell_components[target_location].table is not None:
            target_machine = self.cell_components[target_location]
            target_machine.table.set_table_clamp(clamped)
        #     manifold = self.cell_components[target_location].table_clamp_manifold
        #     return manifold.change_valve_state(manifold.schunkclamp, manifold.unclamp if not clamped else manifold.clamp)

        # elif "table_unclamp" in self.io_point_configuration['output'].keys() and not self.simulate_io:
        #     bound_axis_label = self.io_point_configuration['output']['table_unclamp']['bound_axis_label']
        #     point_address = self.io_point_configuration['output']['table_unclamp']['point_address']
        #     self.axes[bound_axis_label].set_IO_point(port=self.port,
        #                                              io_point_address=point_address,
        #                                              io_point_value=not clamped)

            return True
        return False

    '''
    '''
    ### EXTERNAL MODE SWITCHES
    async def stop_mode(self) -> MotionControllerCommandResponse:
        self.stop_axes()
        if await self.wait_for_motion_stop(self.machine_setup_configuration['state_transition_timeout']):
            self.to_stop_mode()
            #self.run_state_method = self.run_manual_mode
            return MotionControllerCommandResponse(success=True, response_string="Switched to stop mode")
        else:
            return MotionControllerCommandResponse(success=False, response_string="Timed out on mode switch to STOP waiting for axis stop")

    async def manual_mode(self) -> MotionControllerCommandResponse:
        self.stop_axes()
        if await self.wait_for_motion_stop(self.machine_setup_configuration['state_transition_timeout']):
            self.to_manual_mode()
            #self.run_state_method = self.run_manual_mode
            return MotionControllerCommandResponse(success=True, response_string="Switched to manual mode")
        else:
            return MotionControllerCommandResponse(success=False,
                                                   response_string="Timed out on mode switch to MANUAL waiting for axis stop")

    async def abort(self) -> MotionControllerCommandResponse:
        self.abort_sequence()
        self.stop_axes()
        self.to_error_mode()
        if await self.wait_for_motion_stop(self.machine_setup_configuration['state_transition_timeout']):
            #self.to_ERROR()
            # self.run_state_method = self.run_manual_mode
            return MotionControllerCommandResponse(success=True, response_string="Sequence aborted, switching to ERROR mode")
        else:
            return MotionControllerCommandResponse(success=False,
                                                   response_string="Timed out on mode switch to ERROR waiting for axis stop")

    async def sequence_mode(self) -> MotionControllerCommandResponse:
        if not self.is_state(state=MotionControllerStates.STOPPED, model=self):
            return MotionControllerCommandResponse(success=False, response_string="Motion controller must be in STOP mode to begin sequence")

        try:
            self.to_sequence_mode()
            return MotionControllerCommandResponse(success=True,
                                                   response_string="Motion controller began sequence mode")
        except Exception as e:
            a=5

    def abort_sequence(self):
        if self.sequence_state_machine is not None:
            self.sequence_state_machine.sequencing_to_abort(self)
        
        #Empty sequence queue
        while not self.sequence_queue.empty():
            self.sequence_queue.get()
            #self.to_ERROR()


    def check_manual_transition(self):
        if self.check_motion_stopped():
            return True
        else:
            return False

    '''
    '''
    ### INTERNAL STATE CHANGES
    def internal_stop_mode_change(self):
        self.to_stop_mode()

    '''
    '''
    ### ENTRY METHODS
    def enter_stop_mode(self):
        #self.stop_axes()
        self.stop_axes()
        #self.sequence_state_machine = None
        self.sequence_state_machine = None
        self.run_state_method = self.run_stop_mode

    def enter_manual_mode(self):
        self.run_state_method = self.run_manual_mode

    def enter_error_mode(self):
        self.run_state_method = self.run_error_mode

    def enter_sequence_mode(self):
        #self.sequence_state_machine.to_BEGIN()
        self.run_state_method = self.run_sequence_mode

    @staticmethod
    def set_motion_position_target(motion_controller: object):
        for axis_function, waypoint_label in motion_controller.current_sequence_waypoints[motion_controller.sequence_state_machine.state].items():
            if axis_function in motion_controller.axis_function_map.keys() and waypoint_label is not None:# and not motion_controller.simulate_axes:
                target_axis = motion_controller.axes[motion_controller.axis_function_map[axis_function]]

                #Find waypoint
                target_position = None
                if waypoint_label in motion_controller.current_machine_sequence.__dict__.keys():
                    target_position = getattr(motion_controller.current_machine_sequence, waypoint_label)
                    target_position_label = waypoint_label
                elif motion_controller.current_machine_sequence.target_location is not None:
                    if motion_controller.current_machine_sequence.target_location in motion_controller.rack_locations:
                        calibration_table = motion_controller.rack_locations
                    elif motion_controller.current_machine_sequence.target_location in motion_controller.machine_locations:
                        calibration_table = motion_controller.machine_locations
                    else:
                        raise MotionControllerException

                    calibration_table_waypoint = '_'.join([axis_function, waypoint_label])
                    if calibration_table_waypoint in calibration_table[motion_controller.current_machine_sequence.target_location].keys():
                        target_position = calibration_table[motion_controller.current_machine_sequence.target_location][calibration_table_waypoint]
                        target_position_label = calibration_table_waypoint

                if target_position is None:
                    target_position = motion_controller.calibration_defaults['_'.join(['default', axis_function, waypoint_label])]
                    target_position_label = '_'.join(['default', axis_function, waypoint_label])

                # target_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                #     target_position=getattr(motion_controller.current_machine_sequence, waypoint_label)))
                logger.info("Sequence state {} commanding axis {} to position {} = {}".format(
                    motion_controller.sequence_state_machine.state.name,
                    target_axis.axis_label,
                    target_position_label,
                    str(target_position)
                ))
                target_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                    target_position=target_position))
                target_axis.position_absolute(port=motion_controller.port)
                target_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def check_motion_position_target_reached(motion_controller: object) -> bool:
        position_reached = True
        for axis_function, waypoint_label in motion_controller.current_sequence_waypoints[motion_controller.sequence_state_machine.state].items():
            if axis_function in motion_controller.axis_function_map.keys() and waypoint_label is not None and not motion_controller.simulate_axes:
                target_axis = motion_controller.axes[motion_controller.axis_function_map[axis_function]]
                #position_reached &= target_axis.in_position(getattr(motion_controller.current_machine_sequence, waypoint_label))
                position_reached &= target_axis.in_position(target_axis.target_position)

        if position_reached:
            for axis_function, waypoint_label in motion_controller.current_sequence_waypoints[motion_controller.sequence_state_machine.state].items():
                if axis_function in motion_controller.axis_function_map.keys() and waypoint_label is not None:
                    target_axis = motion_controller.axes[motion_controller.axis_function_map[axis_function]]
                    logger.info("Axis {} reached target position {} = {}".format(
                        target_axis.axis_label,
                        waypoint_label,
                        target_axis.target_position
                    ))

        return position_reached

    @staticmethod
    def set_traverse_position_target(motion_controller: object):
        if 'traverse' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.traverse_end is not None and not motion_controller.simulate_axes:
            traverse_axis = motion_controller.axes[motion_controller.axis_function_map['traverse']]
            traverse_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=motion_controller.current_machine_sequence.traverse_end))
            traverse_axis.position_absolute(port=motion_controller.port)
            traverse_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def set_pallet_position_target(motion_controller: object):
        if 'swing' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.swing_end is not None and not motion_controller.simulate_axes:
            swing_axis = motion_controller.axes[motion_controller.axis_function_map['swing']]
            swing_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=motion_controller.current_machine_sequence.swing_end))
            swing_axis.position_absolute(port=motion_controller.port)
            swing_axis.execute_move(port=motion_controller.port)

        if 'vertical' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.vertical_end is not None and not motion_controller.simulate_axes:
            vertical_axis = motion_controller.axes[motion_controller.axis_function_map['vertical']]
            vertical_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=motion_controller.current_machine_sequence.vertical_end + motion_controller.machine_setup_configuration['pallet_lift_height_mm']))
            vertical_axis.position_absolute(port=motion_controller.port)
            vertical_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def set_reach_position_target(motion_controller: object):
        if motion_controller.state is MotionControllerStates.SEQUENCING_TO_PLACE:
            a=5

        if 'reach' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.reach_end is not None and not motion_controller.simulate_axes:
            reach_axis = motion_controller.axes[motion_controller.axis_function_map['reach']]
            reach_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=motion_controller.current_machine_sequence.reach_end))
            reach_axis.position_absolute(port=motion_controller.port)
            reach_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def set_withdrawal_position_target(motion_controller: object):
        if motion_controller.state is MotionControllerStates.SEQUENCING_TO_PLACE:
            a=5

        if 'reach' in motion_controller.axis_function_map.keys() and not motion_controller.simulate_axes:
            reach_axis = motion_controller.axes[motion_controller.axis_function_map['reach']]
            reach_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=reach_axis.safe_position))
            reach_axis.position_absolute(port=motion_controller.port)
            reach_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def set_safe_position_target(motion_controller: object):
        a=5
        if 'reach' in motion_controller.axis_function_map.keys() and not motion_controller.simulate_axes:
            reach_axis = motion_controller.axes[motion_controller.axis_function_map['reach']]
            reach_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=reach_axis.safe_position))
            reach_axis.position_absolute(port=motion_controller.port)
            reach_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def set_vertical_lift_position_target(motion_controller: object):
        if 'vertical' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.vertical_end is not None and not motion_controller.simulate_axes:
            vertical_axis = motion_controller.axes[motion_controller.axis_function_map['vertical']]
            vertical_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=vertical_axis.current_position + motion_controller.machine_setup_configuration['pallet_lift_height_mm']))
            vertical_axis.position_absolute(port=motion_controller.port)
            vertical_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def set_vertical_lower_position_target(motion_controller: object):
        if 'vertical' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.vertical_end is not None and not motion_controller.simulate_axes:
            vertical_axis = motion_controller.axes[motion_controller.axis_function_map['vertical']]
            vertical_axis.update_axis_commands(port=motion_controller.port, command=AxisMotionCommand(
                target_position=vertical_axis.current_position - motion_controller.machine_setup_configuration['pallet_lift_height_mm']))
            vertical_axis.position_absolute(port=motion_controller.port)
            vertical_axis.execute_move(port=motion_controller.port)

    @staticmethod
    def enter_sequence_error_mode(motion_controller: object):
        #motion_controller.
        pass

    @staticmethod
    def place_pallet(motion_controller: object):
        motion_controller.set_pallet_clamp(clamped=False)

    @staticmethod
    def pick_pallet(motion_controller: object):
        motion_controller.set_pallet_clamp(clamped=True)

    ### CONDITION METHODS
    '''
    '''
    @staticmethod
    def traverse_target_reached(motion_controller: object = None) -> bool:
        if 'traverse' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.traverse_end is not None and not motion_controller.simulate_axes:
            traverse_axis = motion_controller.axes[motion_controller.axis_function_map['traverse']]
            return traverse_axis.in_position(motion_controller.current_machine_sequence.traverse_end)
        return True

    @staticmethod
    def pallet_position_target_reached(motion_controller: object) -> bool:
        position_reached = True
        if 'swing' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.swing_end is not None:
            swing_axis = motion_controller.axes[motion_controller.axis_function_map['swing']]
            position_reached &= swing_axis.in_position(motion_controller.current_machine_sequence.swing_end)

        if 'vertical' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.vertical_end is not None:
            vertical_axis = motion_controller.axes[motion_controller.axis_function_map['vertical']]
            position_reached &= vertical_axis.in_position(motion_controller.current_machine_sequence.vertical_end)

        return position_reached

    @staticmethod
    def reach_target_reached(motion_controller: object) -> bool:
        if 'reach' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.reach_end is not None and not motion_controller.simulate_axes:
            reach_axis = motion_controller.axes[motion_controller.axis_function_map['reach']]
            return reach_axis.in_position(motion_controller.current_machine_sequence.reach_end)
        return True

    @staticmethod
    def safe_position_target_reached(motion_controller: object) -> bool:
        #return False
        if 'reach' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.reach_end is not None and not motion_controller.simulate_axes:
            reach_axis = motion_controller.axes[motion_controller.axis_function_map['reach']]
            return reach_axis.in_position(reach_axis.safe_position)
        return True

    @staticmethod
    def pallet_placed(motion_controller: object) -> bool:
        return True

    @staticmethod
    def pallet_picked(motion_controller: object) -> bool:
        return True

    @staticmethod
    def pallet_withdrawn(motion_controller: object) -> bool:
        #return True
        if 'reach' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.reach_end is not None and not motion_controller.simulate_axes:
            reach_axis = motion_controller.axes[motion_controller.axis_function_map['reach']]
            return reach_axis.in_position(reach_axis.safe_position)
        return True

    @staticmethod
    def pallet_raised(motion_controller: object) -> bool:
        a=5
        if 'vertical' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.vertical_end is not None:
            vertical_axis = motion_controller.axes[motion_controller.axis_function_map['vertical']]
            return vertical_axis.in_position(motion_controller.current_machine_sequence.vertical_end + motion_controller.pallet_lift_height_mm)

        return True

    @staticmethod
    def pallet_lowered(motion_controller: object) -> bool:
        a=5
        if 'vertical' in motion_controller.axis_function_map.keys() and motion_controller.current_machine_sequence.vertical_end is not None:
            vertical_axis = motion_controller.axes[motion_controller.axis_function_map['vertical']]
            return vertical_axis.in_position(motion_controller.current_machine_sequence.vertical_end)

        return True

    @staticmethod
    def check_machine_ready_for_insertion(motion_controller: object) -> bool:
        target_location = motion_controller.current_machine_sequence.target_location
        ready_for_insertion = True
        if target_location in motion_controller.cell_components.keys() and type(motion_controller.cell_components[target_location]) == MachineTool:
            if not motion_controller.simulate_io and motion_controller.cell_components[target_location].door is not None:
                ready_for_insertion &= motion_controller.cell_components[target_location].door.opened()
            if not motion_controller.simulate_io and motion_controller.cell_components[target_location].table is not None:
                motion_controller.set_table_clamp(False)
                ready_for_insertion &= motion_controller.cell_components[target_location].table.unclamped()
        return ready_for_insertion


    ### PREPARATION METHODS
    '''
    '''
    @staticmethod
    def update_sequence_state_history(motion_controller: object):
        if len(motion_controller.sequence_state_history) > 0:
            logger.info("Sequence state machine left state {} and entered state {}, while motion controller in state {}".format(
                motion_controller.sequence_state_history[-1], motion_controller.sequence_state_machine.state.name, motion_controller.state.name))
        else:
            logger.info("Sequence state machine entered state {}, while motion controller in state {}".format(motion_controller.sequence_state_machine.state.name, motion_controller.state.name))
        motion_controller.sequence_state_history.append(motion_controller.sequence_state_machine.state)

    @staticmethod
    def prepare_to_insert_pallet(motion_controller: object):
        #motion_controller.set_table_clamp(False)
        a=5
        pass

    @staticmethod
    def prepare_to_withdraw_pallet(motion_controller: object):
        #motion_controller.set_table_clamp(True)
        a=5
        pass

    @staticmethod
    def prepare_to_pick_pallet(motion_controller: object):
        motion_controller.set_pallet_clamp(False)
        motion_controller.set_table_clamp(False)

    @staticmethod
    def prepare_to_place_pallet(motion_controller: object):
        #motion_controller.set_pallet_clamp(False)
        motion_controller.set_table_clamp(False)
        #motion_controller.current_machine_sequence.destination
        #if target_location in self.cell_components.keys() and type(self.cell_components[target_location]) == MachineTool:


    @staticmethod
    def ready_machine_for_operation(motion_controller: object):
        motion_controller.set_table_clamp(True)

    ### TEARDOWN METHODS
    '''
    '''
    @staticmethod
    def remove_last_transition(motion_controller: object):
        # if len(motion_controller.sequence_state_history) > 1:
        #     logger.info("Sequence state machine left state {}".format(motion_controller.sequence_state_history[-2]))
        #     motion_controller.sequence_state_machine.remove_transition(trigger='next_sequence_step', source=motion_controller.sequence_state_history[-2], dest=motion_controller.sequence_state_history[-1])
        return

    '''
    '''
    ### RUN METHODS
    @logger.catch
    def run_sequence_mode(self):
        if self.sequence_state_machine is not None:
            #print("In sequencing mode with {} sequence, current state is {}".format(str(self.current_machine_sequence.sequence_type), str(self.sequence_state_machine.state)))
            pass
        if self.sequence_state_machine is None or self.sequence_state_machine.is_state(state=SequenceStates.DONE, model=self.sequence_state_machine):
            if not self.sequence_queue.empty():
                self.start_machine_sequence()
            else:
                self.to_stop_mode()
        elif self.sequence_state_machine.is_state(state=SequenceStates.ERROR, model=self.sequence_state_machine):
            print("Sequence SM run method encountered ERROR")
        else:
            self.sequence_state_machine.next_sequence_step(self)

        #TODO: CHECK AXIS POSITIONS ARE CORRECT AND IF NOT ABORT

    def run_manual_mode(self):
        if not all([axis.axis_running for axis in self.axes.values()]) and not self.motion_queue.empty():
            # All axes are finished, so load the next command to the drives
            motion_update_complete = True
            motion_command = self.motion_queue.get_nowait()
            for axis_field in fields(motion_command):
                axis_label = axis_field.name
                axis_command = getattr(motion_command, axis_label)
                if axis_command is not None:
                    motion_update_complete &= self.axes[axis_label].update_axis_commands(self.port,
                                                                                         axis_command,
                                                                                         multiline=False)
                    motion_update_complete &= self.axes[axis_label].position_absolute(self.port)

            if not motion_update_complete:
                logger.error("Motion update failed")

            if (self.machine_setup_configuration['plc_installed']):
                raise NotImplementedError
            else:
                for axis_field in fields(motion_command):
                    axis_command = getattr(motion_command, axis_field.name)
                    if axis_command is not None:
                        self.axes[axis_field.name].execute_move(self.port)

    def run_stop_mode(self):
        pass

    def run_error_mode(self):
        pass

    ### SEQUENCE SETUP
    def create_sequencing_model(self, transitions: List[Dict]) -> Machine:
        sequence_state_machine = Machine(states=SEQUENCE_STATES, initial=SequenceStates.DONE)

        sequence_state_machine.add_transitions(transitions)
        [setattr(sequence_state_machine, callback, getattr(self, callback)) for callback in MACHINE_CALLBACKS]

        return sequence_state_machine

    def create_sequence_waypoints(self, transitions: List[Dict]) -> Dict[str, Dict[str, float]]:
        waypoints = {}
        for transition in transitions:
            if transition['dest'] in SEQUENCE_WAYPOINTS:
                waypoints[transition['dest']] = SEQUENCE_WAYPOINTS[transition['dest']]

        return waypoints

    def start_machine_sequence(self):
        sequence = self.sequence_queue.get()

        if sequence.sequence_type == SequenceTypes.PICK:
            self.sequence_state_machine = self.create_sequencing_model(transitions=SEQUENCE_PICK_TRANSITIONS)
            waypoints = self.create_sequence_waypoints(transitions=SEQUENCE_PICK_TRANSITIONS)
        elif sequence.sequence_type == SequenceTypes.PLACE:
            self.sequence_state_machine = self.create_sequencing_model(transitions=SEQUENCE_PLACE_TRANSITIONS)
            waypoints = self.create_sequence_waypoints(transitions=SEQUENCE_PLACE_TRANSITIONS)
        else:
            raise NotImplementedError

        self.current_machine_sequence = sequence
        self.current_sequence_waypoints = waypoints
        self.sequence_state_machine.to_BEGIN()


    def enqueue_machine_sequence(self, sequence: MachineSequence) -> MotionControllerCommandResponse:
        if not self.check_limits(sequence):
            return MotionControllerCommandResponse(success=False,
                                                   response_string="Desired sequence violates axis limits")

        if sequence.sequence_type != SequenceTypes.PICK and sequence.sequence_type != SequenceTypes.PLACE:
            return MotionControllerCommandResponse(success=False, response_string="Unknown sequence of type {}".format(str(sequence.sequence_type)))

        if sequence.target_location is None:
            sequence.clearance_height = self.default_calibration_locations['clearance_height']
        elif sequence.target_location in self.rack_locations.keys():
            sequence.clearance_height = self.rack_locations[sequence.target_location][
                'clearance_height'] if 'clearance_height' in self.rack_locations[sequence.target_location] else \
            self.default_calibration_locations['clearance_height']
        elif sequence.target_location in self.machine_locations.keys():
            sequence.clearance_height = self.machine_locations[sequence.target_location][
                'clearance_height'] if 'clearance_height' in self.machine_locations[sequence.target_location] else \
            self.default_calibration_locations['clearance_height']
        else:
            return MotionControllerCommandResponse(success=False, response_string="Unknown target location {}".format(
                sequence.target_location))

        sequence.sequence_id = self.sequence_id
        self.sequence_id += 1
        self.sequence_queue.put(sequence)
        return MotionControllerCommandResponse(success=True, response_string="Enqueued machine sequence {}".format(sequence.sequence_id))


    def enqueue_positioning_command(self, command: RobotMotionCommand) -> MotionControllerCommandResponse:
        if self.state == MotionControllerStates.MANUAL:

            command.command_id = self.motion_command_id
            command.move_id = self.motion_command_id

            self.motion_queue.put(command)
            self.motion_command_id += 1

            return MotionControllerCommandResponse(success=True, response_string="Enqueued motion command " + str(command.command_id))
        else:
            return MotionControllerCommandResponse(success=False, response_string="Must be in MANUAL mode to enqueue motion commands")

    def validate_machine_command(self, command: RobotCommand) -> MotionControllerCommandResponse:
        if command.command_type == ExternalCommandType.SET_CELL_SUBCOMPONENT_STATE:
            success = True
            if command.cell_component not in self.cell_components.keys():
                success = False
                response_string=f'Cell component {command.cell_component} not installed in cell'
                #return MotionControllerCommandResponse(success=False, response_string=f"Cell component {command.cell_component} not installed in cell")
            if command.cell_subcomponent not in self.cell_components[command.cell_component].__dict__.keys():
                success = False
                response_string=f'Component {command.cell_component} has no subcomponent \'{command.cell_subcomponent}\' configured'
                #return MotionControllerCommandResponse(success=False, response_string=f'Component {command.cell_component} has no subcomponent {command.cell_subcomponent} configured')
            
            if command.cell_subcomponent == 'table':
                if command.state not in getattr(self.cell_components[command.cell_component], command.cell_subcomponent).valid_states:
                    success = False
                    response_string = f'Must supply one of: ' + ', '.join([('\'' + state + '\'') for state in getattr(self.cell_components[command.cell_component], command.cell_subcomponent).valid_states]) + f' to \'subcomponent_state\', not {command.state}'
                    #\'clamp\' or \'unclamp\' to \'table_state\', not {command.state}'
                    #response_string = f'Must supply either \'clamp\' or \'unclamp\' to \'table_state\', not {command.state}'
            
            if not success:
                logger.info(f'Rejected command type {command.command_type} for reason: {response_string}')
                return MotionControllerCommandResponse(success=False, response_string='COMMAND VALIDATION FALIED: ' + response_string)
            #return MotionControllerCommandResponse(success=True, response_string="command validated")
        return MotionControllerCommandResponse(success=True, response_string="command validated")

    def enqueue_machine_command(self, command: RobotCommand) -> MotionControllerCommandResponse:
        if self.state == MotionControllerStates.MANUAL or self.state == MotionControllerStates.STOPPED or command.emergency == True:
            validation = self.validate_machine_command(command)
            if not validation.success:
                return validation

            command.command_id = self.machine_command_id
            self.machine_command_id += 1

            self.machine_command_queue.put(command)
            logger.info(f'Enqueued command type {command.command_type}: {command}')
            return MotionControllerCommandResponse(success=True, response_string="Enqueued command " + str(command.command_id))
        else:
            return MotionControllerCommandResponse(success=False, response_string="Machine must be in MANUAL mode to enqueue commands")

    def check_limits(self, command: Union[MachineSequence, RobotMotionCommand]) -> MotionControllerCommandResponse:
        for axis_field in fields(command):
            if type(command) == RobotMotionCommand:
                axis_label = axis_field.name
                if getattr(command, axis_label) is None:
                    continue
                elif axis_label not in self.axes_installed:
                    return MotionControllerCommandResponse(success=False, response_string="Axis {} commanded for motion, but this axis is not installed".format(axis_label))

                target_position = getattr(command, axis_label).target_position
                target_velocity = getattr(command, axis_label).target_velocity
                target_acceleration = getattr(command, axis_label).target_acceleration
                target_deceleration = getattr(command, axis_label).target_deceleration
                target_jerk = getattr(command, axis_label).target_jerk
                target_deceleration_jerk = getattr(command, axis_label).target_deceleration_jerk
            elif type(command) == MachineSequence:
                if getattr(command, axis_field.name) is None:
                    continue

                axis_function = axis_field.name.replace("_end", '')
                if axis_function not in self.axis_function_map.keys():
                    return MotionControllerCommandResponse(success=False, response_string="{} axis needed for sequence, but this axis is not installed".format(axis_function))
                axis_label = self.axis_function_map[axis_function]

                if axis_label not in self.axes_installed:
                    return MotionControllerCommandResponse(success=False,
                                                           response_string="Axis {} commanded for sequencing, but this axis is not installed".format(
                                                               axis_label))

                target_position = getattr(command, axis_field.name)
                target_velocity = 0
                target_acceleration = 0
                target_deceleration = 0
                target_jerk = 0
                target_deceleration_jerk = 0
            else:
                raise NotImplementedError


            if target_position < self.axes[axis_label].negative_limit:
                return MotionControllerCommandResponse(
                    success=False, response_string="Axis {} command past negative limit of {}".format(axis_label, str(
                        self.axes[axis_label].negative_limit)))
            if target_position > self.axes[axis_label].positive_limit:
                return MotionControllerCommandResponse(
                    success=False, response_string="Axis {} command past positive limit of {}".format(axis_label, str(
                        self.axes[axis_label].positive_limit)))
            if target_velocity is not None and target_velocity > self.axes[axis_label].maximum_velocity:
                return MotionControllerCommandResponse(
                    success=False, response_string="Axis {} command past velocity limit of {}".format(axis_label, str(
                        self.axes[axis_label].maximum_velocity)))
            if target_acceleration is not None and target_acceleration > self.axes[axis_label].maximum_acceleration:
                return MotionControllerCommandResponse(
                    success=False, response_string="Axis {} command past acceleration limit of {}".format(axis_label, str(
                        self.axes[axis_label].maximum_acceleration)))
            if target_deceleration is not None and target_deceleration > self.axes[axis_label].maximum_acceleration:
                return MotionControllerCommandResponse(
                    success=False, response_string="Axis {} command past deceleration limit of {}".format(axis_label, str(
                        self.axes[axis_label].maximum_acceleration)))
            if target_jerk is not None and target_jerk > self.axes[axis_label].maximum_jerk:
                return MotionControllerCommandResponse(
                    success=False, response_string="Axis {} command past jerk limit of {}".format(axis_label, str(
                        self.axes[axis_label].maximum_jerk)))
            if target_deceleration_jerk is not None and target_deceleration_jerk > self.axes[
                axis_label].maximum_jerk:
                return MotionControllerCommandResponse(
                    success=False, response_string="Axis {} command past deceleration jerk limit of {}".format(axis_label, str(
                        self.axes[axis_label].maximum_jerk)))

        return MotionControllerCommandResponse(success=True, response_string="Limit check passed")

    def process_machine_command(self, command: RobotCommand):
        ### GENERAL COMMANDS
        if command.command_type == ExternalCommandType.SET_IO_POINT:
            if command.point_address is not None:
                target_axis_label = command.target_axis_label
                self.axes[target_axis_label].set_IO_point(port=self.port,
                                                          io_point_address=command.point_address,
                                                          io_point_value=command.point_state)
            else:
                bound_axis_address = self.io_point_configuration['output'][command.point_function]['bound_axis_label']
                point_address = self.io_point_configuration['output'][command.point_function]['point_address']
                self.axes[bound_axis_address].set_IO_point(port=self.port,
                                                           io_point_address=point_address,
                                                           io_point_value=command.point_state)
        elif command.command_type == ExternalCommandType.SET_CELL_SUBCOMPONENT_STATE:
            target_component = self.cell_components[command.cell_component]
            target_subcomponent = getattr(target_component, command.cell_subcomponent)
            target_subcomponent.set_state(command.state)

        else:
            ### AXIS SPECIFIC COMMANDS
            for command_field in fields(command):
                if command_field.type != AxisCommand or getattr(command, command_field.name) is None:
                    continue

                if command.command_type == ExternalCommandType.ENABLE:
                    self.axes[command_field.name].enable_axis(self.port)
                    # for axis_label in self.axes.keys():
                    #     self.axes[axis_label].enable_axis(self.port)
                elif command.command_type == ExternalCommandType.HOME:
                    if command_field.name in self.axes.keys():
                        self.axes[command_field.name].home_axis(self.port)
                        self.axes[command_field.name].execute_move(self.port)
                    else:
                        print("UNKNOWN AXIS REQUESTED FOR HOMING")
                elif command.command_type == ExternalCommandType.RESET:
                    self.axes[command_field.name].reset_axis(port=self.port)
                elif command.command_type == ExternalCommandType.DISABLE:
                    self.axes[command_field.name].disable_axis(port=self.port)
                    #raise NotImplementedError
                elif command.command_type == ExternalCommandType.STOP:
                    #self.axes[command.axis_label].stop_axis(port=self.port)
                    self.axes[command_field.name].stop_axis(port=self.port)



    def run(self):
        while self.run_flag:
            for axis_label, axis in self.axes.items():
                with self.data_lock:
                    axis.update_axis(self.port, multiline=True)

                time.sleep(self.communication_configuration['update_delay'])
            self.log_axis_motion()

            if not self.machine_command_queue.empty():
                self.process_machine_command(self.machine_command_queue.get())

            if self.run_state_method is not None:
                self.run_state_method()