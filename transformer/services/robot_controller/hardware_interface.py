import serial, time

from typing import Any
from dataclasses import field, fields
from loguru import logger
from services.robot_controller.messages import *

class Compax3ErrorCode(Enum):
    NO_ERROR = 1

@dataclass
class Compax3M:
    communication_configuration: Dict

    axis_address: int
    axis_label: str
    axis_function: str
    comm_format: str

    default_velocity: float = 0
    default_acceleration: float = 0
    default_deceleration: float = 0
    default_jerk: float = 0
    default_deceleration_jerk: float = 0

    units: str = 'mm'
    position_deadband: float = 0
    safe_position: float = 0
    zero_offset: float = 0
    scale_per_rev: float = 0
    feedback_resolution: float = 65536
    negative_limit: float = 0
    positive_limit: float = 0

    maximum_velocity: float = 0
    maximum_acceleration: float = 0
    maximum_jerk: float = 0
    maximum_update_attempts: int = 0

    home_position_motor: float = 0

    current_position: float = 0
    current_velocity: float = 0
    current_acceleration: float = 0
    current_jerk: float = 0
    current_current: float = 0

    target_position: ClassVar[float] = None
    target_velocity: ClassVar[float] = None
    target_acceleration: ClassVar[float] = None
    target_deceleration: ClassVar[float] = None
    target_jerk: ClassVar[float] = None
    target_deceleration_jerk: ClassVar[float] = None

    number_of_inputs: int = 0
    number_of_outputs: int = 0
    number_of_status_bits: int = 0
    enabled: bool = False
    axis_error: bool = False
    current_error_code: int = 0
    previous_error_code: Compax3ErrorCode = 1#Compax3ErrorCode.NO_ERROR
    axis_running: bool = False
    io_status_bits: int = 0

    #inputs: List = field(default_factory=list)
    inputs: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    outputs: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    status_bit_definitions: List = field(default_factory=list)
    drive_status_bits: Dict[str, Dict[str, bool]] = field(default_factory=dict)
    #drive_status_bits: Dict[str, Dict[str, bool]] = field(default_factory={'power_enabled': False, 'error status': False})
    #outputs: List = field(default_factory=list)

    motion_logger: logger = None
    update_time: ClassVar[float] = -1

    def __post_init__(self):
        #self.inputs = 8*[False]
        #self.outputs = 4*[False]
        self.motion_logger = logger.bind(motion=True)

        #fill in missing IO points
        defined_input_addresses = []
        for _, input in self.inputs.items():
            defined_input_addresses.append(input['point_address'])
        
        for point_address in range(self.number_of_inputs):
            if point_address not in defined_input_addresses:
                self.inputs['input_' + str(point_address)] = {'point_address': point_address}
        
        defined_output_addresses = []
        for _, output in self.outputs.items():
            defined_output_addresses.append(output['point_address'])
        
        for point_address in range(self.number_of_outputs):
            if point_address not in defined_output_addresses:
                self.outputs['input_' + str(point_address)] = {'point_address': point_address}

        #self.motion_logger.trace("X Position, X Velocity, Y Position, Y Velocity")
        pass

    def in_position(self, target: float) -> bool:
        return (not self.axis_running and abs(target - self.current_position) <= self.position_deadband)

    def transform_to_physical_units(self, feedback_counts: int) -> float:
        #print(struct.pack('>I', feedback_counts))
        return feedback_counts / self.feedback_resolution * self.scale_per_rev

    def parse_io_bits(self):
        bit_offset = 0
        outputs = self.number_of_outputs*[False]
        for bit_index in range(0, self.number_of_outputs):
            outputs[bit_index] = bool(self.io_status_bits & (1 << bit_index))
            for output_function, output_params in self.outputs.items():
                if output_params['point_address'] == bit_index:
                    output_params['point_state'] = outputs[bit_index]

        #print(outputs)
        bit_offset += self.number_of_outputs
        inputs = self.number_of_inputs*[False]
        for bit_index in range(0, self.number_of_inputs):
            inputs[bit_index] = bool(self.io_status_bits & (1 << (bit_index + bit_offset)))
            for input_function, input_params in self.inputs.items():
                if input_params['point_address'] == bit_index:
                    input_params['point_state'] = inputs[bit_index]

        bit_offset += self.number_of_inputs
        status_bits = self.number_of_status_bits*[False]
        for bit_index in range(0, self.number_of_status_bits):
            status_bits[bit_index] = bool(self.io_status_bits & (1 << (bit_index + bit_offset)))
            self.drive_status_bits[self.status_bit_definitions[bit_index]] = status_bits[bit_index]
            # for status_param_name, status_param_value in self.inputs.items():
            #     if input_params['point_address'] == bit_index:
            #         input_params['point_state'] = inputs[bit_index]

    def log_errors(self):
        if self.current_error_code != self.previous_error_code and self.current_error_code != Compax3ErrorCode.NO_ERROR.value:
            logger.error(f'Axis {self.axis_label} encountered error code {hex(self.current_error_code)}')
            self.previous_error_code = self.current_error_code
        elif self.current_error_code != self.previous_error_code and self.current_error_code == Compax3ErrorCode.NO_ERROR.value:
            logger.error(f'Axis {self.axis_label} cleared error code {hex(self.previous_error_code)}')
            self.previous_error_code = self.current_error_code
        else:
            #No change in error state
            pass


    def write_message_sequence(self, port: serial.Serial, message_sequence: List[Dict], value_sequence: List = None) -> str:
        if self.comm_format == 'ascii':
            message_payload: str = b''
            if value_sequence is not None:
                for message, value in zip(message_sequence, value_sequence):
                    message_payload += message['message_type'](
                        axis_address=self.axis_address if self.communication_configuration[
                                                              'communication_type'] == 'rs485' else None,
                        set_value=value).Serialize()
                    # set_value=getattr(command, message['command_attribute_name'])).Serialize()
            else:
                for message in message_sequence:
                    message_payload += message['message_type'](
                        axis_address=self.axis_address if self.communication_configuration[
                                                              'communication_type'] == 'rs485' else None).Serialize()
            port.flushInput()
            port.write(message_payload)
            return message_payload
        else:
            # TODO Refactor this to not use list of dicts
            if type(message_sequence[0]) is not dict:
                if value_sequence == None:
                    message_payload = ReadObjectTelegram(
                        [message.object_index for message in message_sequence],
                        [message.object_subindex for message in message_sequence],
                        self.axis_address).Serialize()
                else:
                    if len(message_sequence) > 1:
                        raise NotImplementedError

                    index = message_sequence[0].object_index
                    subindex = message_sequence[0].object_subindex
                    message_payload = WriteObjectTelegram(
                        index=index,
                        subindex=subindex,
                        data=value_sequence[0],
                        axis_address=self.axis_address
                    ).payload
            else:
                if value_sequence == None:
                    message_payload = ReadObjectTelegram(
                        [message['message_type'].object_index for message in message_sequence],
                        [message['message_type'].object_subindex for message in message_sequence],
                        self.axis_address).Serialize()
                else:
                    if len(message_sequence) > 1:
                        raise NotImplementedError

                    index = message_sequence[0]['message_type'].object_index
                    subindex = message_sequence[0]['message_type'].object_subindex
                    message_payload = WriteObjectTelegram(
                        index=index,
                        subindex=subindex,
                        data=value_sequence[0],
                        axis_address=self.axis_address
                    ).payload

            port.flushInput()
            port.write(message_payload)
            #for payload in message_payloads:

                #time.sleep(self.communication_configuration['rs485_intertransmission_delay'])

    def receive_payload(self, port: serial.Serial, message_sequence: List[Dict], processing_callback: Any, callback_args: Dict = {}) -> MessageProcessed:#, callback_args: Dict):
        if self.comm_format == 'ascii':
            processing_status = MessageProcessed()
            buffer = b''
            attempts = 0
            while not processing_status.received and attempts <= self.communication_configuration['receive_attempts']:
                buffer += port.read(self.communication_configuration['receive_chunk_size'])
                processing_status = processing_callback(buffer, message_sequence, **callback_args)
                #multiline_payload_processed = processing_callback(buffer, message_sequence, **callback_args)
                attempts += 1

                if len(buffer) == 0:
                    #wait for more data
                    time.sleep(self.communication_configuration['receive_delay'])

            if attempts >= self.communication_configuration['receive_attempts']:
                raise serial.SerialException("Receive failed")

            #print("returning processing status")
            return processing_status
        else:
            buffer = b''
            attempts = 0
            while attempts <= self.communication_configuration['receive_attempts']:
                buffer += port.read(self.communication_configuration['receive_chunk_size'])
                attempts += 1

                if len(buffer) == 0:
                    # wait for more data
                    time.sleep(self.communication_configuration['receive_delay'])
                else:
                    #switch on the start byte
                    if buffer[0] == ResponseStartCodes.RESPONSE.value:
                        response = ReadObjectResponseTelegram(received_data=buffer, expected_message_sequence=message_sequence)
                        processing_callback = self.process_received_read_object_telegram
                    elif buffer[0] == ResponseStartCodes.ACK.value:
                        response = AckTelegram(received_data=buffer)
                        #raise NotImplementedError
                    elif buffer[0] == ResponseStartCodes.NACK.value:
                        raise NotImplementedError
                    else:
                        raise AxisCommunicationException("Unknown start byte")

                    if response.success:
                        if processing_callback is not None:
                            processing_callback(telegram=response, message_sequence=message_sequence)
                        return MessageProcessed(received=True, correct=True, response=buffer, received_telegram=response)
                    else:
                        time.sleep(self.communication_configuration['receive_delay'])
                        continue

            if attempts >= self.communication_configuration['receive_attempts']:
                raise AxisCommunicationException("Receive failed")


    def guaranteed_write_read(self, port: serial.Serial, message_sequence: List[Dict], process_callback: Any, value_sequence: List = None, callback_args: Dict = {}) -> MessageProcessed:
        update_attempts = 0
        while update_attempts <= self.maximum_update_attempts:
            if self.comm_format == 'ascii':
                try:
                    message_payload = self.write_message_sequence(port=port, message_sequence=message_sequence, value_sequence=value_sequence)
                    processing_status = self.receive_payload(port,
                                                             message_sequence,
                                                             process_callback,
                                                             callback_args)
                    if not processing_status.correct:
                        raise ValueError("Axis {} received incorrect data while processing response to message sequence {}, received {}".format(self.axis_label, str(message_payload.decode().replace('\r', ' ')), processing_status.response))
                    break
                except (serial.SerialException, ValueError) as e:
                    # Retry the transmission
                    if processing_status.response.decode() == '':
                        a=5
                    print('Exception ' + e.args[0])
                    update_attempts += 1
            else:
                try:
                    self.write_message_sequence(port=port, message_sequence=message_sequence, value_sequence=value_sequence)
                    processing_status = self.receive_payload(port=port, message_sequence=message_sequence, processing_callback=None)

                    if processing_status.received and processing_status.correct:
                        break

                except AxisCommunicationException as e:
                    if 'CRC' in e.args[0]:
                        a=5
                        logger.warning('CRC mismatch on update attempt {} for message sequence {}'.format(update_attempts, str(message_sequence)))
                    update_attempts += 1
                    continue


        if update_attempts > self.maximum_update_attempts:
            logger.error("Failed to communicate with {} axis after {} attempts".format(self.axis_label, update_attempts))
            raise AxisCommunicationException("Failed to communicate with {} axis after {} attempts".format(self.axis_label, update_attempts))

        #return processing_status.correct
        return processing_status

    def update_axis(self, port: serial.Serial=None, multiline: bool = False):
        if self.comm_format == 'ascii':
            try:
                if not multiline:
                    for message in update_message_sequence:
                        port.write(message['message_type'](axis_address=self.axis_address if self.communication_configuration['communication_type'] == 'rs485' else None).Serialize())
                        setattr(self, message['attribute_name'], message['message_type'](received_data=port.readline()).get_value)
                else:
                    # First update the current axis status
                    self.guaranteed_write_read(port=port,
                                               message_sequence=update_message_sequence,
                                               process_callback=self.process_received_multiline_get_payload
                                               )


                self.update_time = time.time()
                print(self.update_time)
            except Exception as e:
                #Catch timeout exception
                logger.error("Axis update encountered error {}".format(e.args[0]))
        else:
            try:
                self.guaranteed_write_read(port=port, message_sequence=update_message_sequence, process_callback=None)
                self.update_time = time.time()

                self.parse_io_bits()
                self.log_errors()
            except AxisCommunicationException:
                a=5

            #self.motion_logger.trace(f"Axis {self.axis_label} position is {self.current_position}")
            #print(self.outputs)

    def write_axis_command_sequence(self):
        pass

    def check_command_default_dynamics(self, command: AxisMotionCommand) -> AxisMotionCommand:
        for parameter in fields(command):
            parameter_value = getattr(command, parameter.name)
            if parameter_value is None:
                default_value = getattr(self, command.default_value_map[parameter.name])
                setattr(command, parameter.name, default_value)
        return command


    def update_axis_commands(self, port: serial.Serial, command: AxisMotionCommand, multiline: bool = False):
        if self.comm_format == 'ascii':
            if not multiline:
                raise NotImplementedError
            else:
                ack = self.guaranteed_write_read(port=port,
                                                 message_sequence=command_message_sequence,
                                                 process_callback=self.process_received_multiline_set_acknowledgements,
                                                 value_sequence=[getattr(command, message['command_attribute_name']) for message in command_message_sequence]).correct

                if ack:

                    # read back axis commands to ensure they were written correctly
                    set_values_matched = self.guaranteed_write_read(port=port, message_sequence=command_message_sequence,
                                               process_callback=self.process_received_multiline_set_confirmation_values,
                                               callback_args={'command': command}).correct


                    return set_values_matched
                else:
                    return False
        else:
            if not multiline:
                command = self.check_command_default_dynamics(command)
                success = True
                for message in command_message_sequence:
                    try:
                        success &= self.guaranteed_write_read(port=port,
                                                              message_sequence=[message],
                                                              process_callback=None,
                                                              value_sequence=[
                                                                  getattr(command, message['command_attribute_name'])]).correct

                        #Check that values were written correctly
                        value_check = self.guaranteed_write_read(port=port,
                                                              message_sequence=[message],
                                                              process_callback=None)
                        #tx_success = (getattr(value_check.received_telegram, message['attribute_name']) == getattr(command, message['command_attribute_name']))
                        tx_success = abs((getattr(value_check.received_telegram, message['attribute_name']) - getattr(
                            command, message['command_attribute_name']))) <= self.communication_configuration[
                                         'float_match_tolerance']
                        if tx_success:
                            setattr(self, message['command_attribute_name'], getattr(command, message['command_attribute_name']))

                        success &= tx_success



                    except Exception as e:
                        logger.error("Communication error: " + e.args[0])

                return success
            else:
                raise NotImplementedError

    def enable_axis(self, port: serial.Serial) -> bool:
        if self.comm_format == 'ascii':
            success = self.guaranteed_write_read(port=port,
                                                 message_sequence=enable_message_sequence,
                                                 value_sequence=[1, 1],
                                                 process_callback=self.process_received_multiline_set_acknowledgements).correct
        else:
            success = True
            for message in enable_message_sequence:
                success &= self.guaranteed_write_read(port=port,
                                                      message_sequence=[message],
                                                      value_sequence=[message['message_type'].set_value],
                                                      process_callback=None).correct
        if success:
            logger.info('Enabled {} axis'.format(self.axis_label))
        else:
            logger.error('Enabling of {} axis failed'.format(self.axis_label))
        return success

    def execute_move(self, port: serial.Serial) -> bool:
        if self.comm_format == 'ascii':
            success = self.guaranteed_write_read(port=port,
                                                 message_sequence=execute_message_sequence,
                                                 process_callback=self.process_received_multiline_set_acknowledgements).correct
        else:
            success = self.guaranteed_write_read(port=port,
                                                 message_sequence=execute_message_sequence,
                                                 value_sequence=[1],
                                                 process_callback=None).correct
        return success
        #return self.set_acknowledged

    def position_absolute(self, port: serial.Serial) -> bool:
        if self.comm_format == 'binary':
            for message in absolute_position_message_sequence:
                success = self.guaranteed_write_read(port=port,
                                                     message_sequence=[message],
                                                     value_sequence=[message['message_type'].set_value],
                                                     process_callback=None).correct  # ,
        else:
            raise NotImplementedError
        return success

    def home_axis(self, port: serial.Serial) -> bool:
        if self.comm_format == 'binary':
            success = True
            for message in homing_message_sequence:
                if 'attribute_name' in message.keys():
                    success &= self.guaranteed_write_read(port=port,
                                                          message_sequence=[message],
                                                          value_sequence=[getattr(self, message['attribute_name'])],
                                                          process_callback=None).correct
                else:
                    success &= self.guaranteed_write_read(port=port,
                                                          message_sequence=[message],
                                                          value_sequence=[message['message_type'].set_value],
                                                          process_callback=None).correct
        else:
            raise NotImplementedError
        if success:
            logger.info('Started homing {} axis'.format(self.axis_label))
        else:
            logger.error('Homing of {} axis failed'.format(self.axis_label))
        return success

    def reset_axis(self, port: serial.Serial) -> bool:
        if self.comm_format == 'binary':
            for message in reset_message_sequence:
                success = self.guaranteed_write_read(port=port,
                                                     message_sequence=[message],
                                                     value_sequence=[message['message_type'].set_value],
                                                     process_callback=None).correct  # ,
        else:
            raise NotImplementedError
        if success:
            logger.info('Reset {} axis'.format(self.axis_label))
            self.current_error_code = 0
        else:
            logger.error('Reset of {} axis failed'.format(self.axis_label))
        return success

    def disable_axis(self, port: serial.Serial) -> bool:
        if self.comm_format == 'binary':
            for message in disable_message_sequence:
                success = self.guaranteed_write_read(port=port,
                                                     message_sequence=[message],
                                                     value_sequence=[message['message_type'].set_value],
                                                     process_callback=None).correct  # ,
        else:
            raise NotImplementedError
        if success:
            logger.info('Disabled {} axis'.format(self.axis_label))
        else:
            logger.error('Disabling of {} axis failed'.format(self.axis_label))
        return success

    def stop_axis(self, port: serial.Serial) -> bool:
        if self.comm_format == 'binary':
            for message in stop_message_sequence:
                if message['message_type'].set_value == None:
                    message['message_type'].set_value = getattr(self, message['attribute_name'])
                success = self.guaranteed_write_read(port=port,
                                                     message_sequence=[message],
                                                     value_sequence=[message['message_type'].set_value],
                                                     process_callback=None).correct  # ,
        else:
            raise NotImplementedError

        if success:
            logger.info('Stopped {} axis'.format(self.axis_label))
        else:
            logger.error('Stopping of {} axis failed'.format(self.axis_label))
        return success

    def set_IO_point(self, port: serial.Serial, io_point_address: int, io_point_value: bool) -> bool:
        io_point_value = 1 if io_point_value else 0
        if self.comm_format == 'binary':
            for message_type in io_point_message_sequence:
                message = message_type['message_type'](axis_address=self.axis_label)
                if 'command_attribute_name' in message_type.keys() and message_type['command_attribute_name'] in locals().keys():
                    # message = message['message_type'](axis_address=self.axis_label)
                    message.set_value = locals()[message_type['command_attribute_name']]
                # if message['message_type'].set_value == None:
                #     message['message_type'].set_value = locals()[message['command_attribute_name']]
                success = self.guaranteed_write_read(port=port,
                                                     message_sequence=[message],
                                                     value_sequence=[message.set_value],
                                                     process_callback=None).correct  # ,
        else:
            raise NotImplementedError
        #print('SET AXIS {} IO POINT {} TO {}'.format(self.axis_label, str(io_point_address), str(io_point_value)))
        if success:
            logger.info(
                'Set axis {} IO point {} to {}'.format(self.axis_label, str(io_point_address), str(io_point_value)))
        else:
            logger.error(
                'Seting axis {} IO point {} to {} failed'.format(self.axis_label, str(io_point_address), str(io_point_value)))
        return success

    def acknowledge_command_error(self, port: serial.Serial) -> bool:
        if self.comm_format == 'binary':
            for message in acknowledge_command_error_message_sequence:
                # if message['message_type'].set_value == None:
                #     message['message_type'].set_value = locals()[message['command_attribute_name']]
                success = self.guaranteed_write_read(port=port,
                                                     message_sequence=[message],
                                                     value_sequence=[message['message_type'].set_value],
                                                     process_callback=None).correct
        else:
            raise NotImplementedError
        return success

    def format_received_binary_string(self, received_string: str) -> List:
        if self.comm_format == 'ascii':
            return received_string.decode().strip().split('\r') if not received_string == b'' else []
        else:
            raise NotImplementedError

    def process_received_read_object_telegram(self, telegram: ReadObjectResponseTelegram, message_sequence: List) -> bool:
        try:
            for message in message_sequence:
                if 'transform' in message.keys() and message['transform'] == True:
                    setattr(self, message['attribute_name'], self.transform_to_physical_units(getattr(telegram, message['attribute_name'])))
                else:
                    setattr(self, message['attribute_name'], getattr(telegram, message['attribute_name']))
        except Exception as e:
            a=5


    def process_set_ack_telegram(self, telegram: ReadObjectResponseTelegram, message_sequence: List) -> bool:
        pass