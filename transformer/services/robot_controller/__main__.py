import time, uvicorn, sys, zmq, yaml, os, json, asyncio
from fastapi import FastAPI, WebSocket
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK
from fastapi.middleware.cors import CORSMiddleware
#from pathlib import Path
from loguru import logger


from . import *
from .motion_control import MotionController, MachineSequence, SequenceTypes
from .messages import *
from .api import *

app = FastAPI()

origins = [
    "*"
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)

def load_MES_config(config_file_path: str) -> Dict:
    with open(os.path.join(ROOT_PATH, config_file_path), 'r') as config_file:
        config = yaml.load(config_file, Loader=yaml.Loader)
        return config

def validate_command_axis_labels(command: HTTPCommand) -> List:
    invalid_axis_labels = []
    for axis_label in command.__fields__:
        if axis_label not in motion_controller.axes.keys() and type(getattr(command, axis_label)) == HTTPAxisCommand:
            invalid_axis_labels.append(axis_label)
    return invalid_axis_labels

def convert_to_axis_motion_command(command: HTTPAxisPositioningCommand) -> AxisMotionCommand:
    converted_command = AxisMotionCommand(target_position=None)
    for parameter_name in command.__fields__:
        parameter_value = getattr(command, parameter_name)
        setattr(converted_command, parameter_name, parameter_value)
    return converted_command

def convert_to_robot_motion_command(command: HTTPRobotPositioningCommand) -> RobotMotionCommand:
    converted_command = RobotMotionCommand()
    for axis_name in command.__fields__:
        axis_command = getattr(command, axis_name)
        if axis_command is not None:
            setattr(converted_command, axis_name, convert_to_axis_motion_command(axis_command))
    return converted_command

def convert_to_robot_command(command: HTTPRobotCommand) -> RobotCommand:
    converted_command = RobotCommand()
    for axis_name in command.__fields__:
        axis_command = getattr(command, axis_name)
        if axis_command is not None:
            setattr(converted_command, axis_name, convert_to_axis_motion_command(axis_command))
    return converted_command

def convert_to_robot_axis_command(command: HTTPAxisCommand) -> AxisCommand:
    converted_command = AxisCommand
    for parameter_name in command.__fields__:
        parameter_value = getattr(command, parameter_name)
        setattr(converted_command, parameter_name, parameter_value)
    return converted_command

def handle_motion_command(command: HTTPRobotPositioningCommand) -> HTTPResponse:
    command = convert_to_robot_motion_command(command)
    limit_check = motion_controller.check_limits(command)
    if not limit_check.success:
        return HTTPResponse(error_string=limit_check.response_string)

    response = motion_controller.enqueue_positioning_command(command)
    if not response.success:
        return HTTPResponse(error_string=response.response_string)
    else:
        return HTTPResponse(success_string=response.response_string)

def create_robot_command(axis_label: str, command_type: ExternalCommandType) -> RobotCommand:
    command = RobotCommand(command_type=command_type)
    setattr(command, axis_label, AxisCommand(command_type=command_type))
    return command

'''
'''
### OPERATOR UI STUFF
@app.get("/", tags=["root"])
async def read_root() -> dict:
    return {"message": "3R Robot"}


@app.get("/machine/axis_positions")
async def get_current_positions() -> Dict:
    positions = {}
    for axis_label, axis_object in motion_controller.axes.items():
        positions[axis_label] = axis_object.current_position
    return positions

@app.get("/machine/state")
async def get_current_state() -> Dict:
    if motion_controller.sequence_state_machine is not None:
        return {
            "motion_controller_state": str(motion_controller.state.name),
            "sequencer_state": str(motion_controller.sequence_state_machine.state.name)
        }
    else:
        return {
            "motion_controller_state": motion_controller.state.name
        }

@app.get("/axes/{axis_label}/error")
async def get_axis_error_state(axis_label: str) -> HTTPResponse:
    if axis_label not in motion_controller.axes_installed:
        return HTTPResponse(error_string="Axis {} is not installed")

    with motion_controller.data_lock:
        data = {'error': motion_controller.axes[axis_label].axis_error}
    return HTTPResponse(data=data)

@app.post("/machine/position_command")
async def absolute_positioning_command(command: HTTPRobotPositioningCommand) -> HTTPResponse:
    return handle_motion_command(command)

@app.post("/machine/reset")
async def reset_axes(command: HTTPCommand) -> HTTPResponse:
    axes_reset = []
    for axis_label in motion_controller.axes_installed:
        response = motion_controller.enqueue_machine_command(
                        create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.RESET))
        axes_reset.append(axis_label)
        if not response.success:
            return BooleanHTTPResponse(success=False, response=response.response_string)
    return BooleanHTTPResponse(success=False, response="Resetting axes: " + ", ".join(axes_reset))

@app.post("/machine/enable")
async def enable_axes(command: HTTPCommand) -> BooleanHTTPResponse:
    axes_enabled = []
    for axis_label in motion_controller.axes_installed:
        response = motion_controller.enqueue_machine_command(
                        create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.ENABLE))
        axes_enabled.append(axis_label)
        if not response.success:
            return BooleanHTTPResponse(success=False, response=response.response_string)
    return BooleanHTTPResponse(success=False, response="Enabling axes: " + ", ".join(axes_enabled))

@app.post("/machine/disable")
async def disable_axes(command: HTTPCommand) -> HTTPResponse:
    invalid_axis_labels = validate_command_axis_labels(command)
    if len(invalid_axis_labels) == 0:
        axes_disabled = []
        for axis_label in command.__fields__:
            if type(getattr(command, axis_label)) == HTTPAxisCommand:
                motion_controller.enqueue_machine_command(
                    create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.ENABLE))
            else:
                continue
        return HTTPResponse(success_string="Disabling axes: " + ", ".join(axes_disabled))
    else:
        return HTTPResponse(error_string="Unknown axis labels for disable request: " + ", ".join(invalid_axis_labels))

@app.post("/axes/{axis_label}/reset")
async def reset_axis(axis_label: str, command: HTTPCommand):
    if command.command_arg != 0:
        if axis_label in motion_controller.axes.keys():
            motion_controller.enqueue_machine_command(create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.RESET))
            return HTTPResponse(success_string="Resetting axis {}".format(axis_label))
        else:
            return HTTPResponse(error_string="Unknown axis {} requested for reset".format(axis_label))
    else:
        return HTTPResponse(error_string="Did not reset {} axis".format(axis_label))

@app.post("/axes/{axis_label}/enable")
async def enable_axis(axis_label: str, command: HTTPCommand) -> HTTPResponse:
    if axis_label in motion_controller.axes.keys():
        if command.command_arg == 1:
            #motion_controller.enqueue_machine_command(HTTPRobotCommand(command_type=ExternalCommandType.ENABLE))
            motion_controller.enqueue_machine_command(
                create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.ENABLE))
        return HTTPResponse(success_string="Enabled axis {}".format(axis_label))
    else:
        return HTTPResponse(error_string="Unknown axis {} requested for enable".format(axis_label))

@app.post("/axes/{axis_label}/disable")
async def disable_axis(axis_label: str, command: HTTPCommand) -> HTTPResponse:
    if axis_label in motion_controller.axes.keys():
        if command.command_arg == 1:
            motion_controller.enqueue_machine_command(
                create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.DISABLE))
            # motion_controller.enqueue_machine_command(HTTPRobotCommand(command_type=ExternalCommandType.DISABLE,
            #                                                            axis_label=axis_label))
        return HTTPResponse(success_string="Disabled axis {}".format(axis_label))
    else:
        return HTTPResponse(error_string="Unknown axis {} requested for disable".format(axis_label))

@app.post("/axes/{axis_label}/home")
async def home_axis(axis_label: str, command: HTTPCommand) -> HTTPResponse:
    if command.command_arg == 1:
        if axis_label in motion_controller.axes.keys():
            #motion_controller.enqueue_machine_command(HTTPRobotCommand(command_type=ExternalCommandType.HOME, axis_label=axis_label))
            motion_controller.enqueue_machine_command(
                create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.HOME))
            return HTTPResponse(success_string="Enqueued homing command for axis {}".format(axis_label))
        else:
            return HTTPResponse(error_string="Unknown axis {} requested for homing".format(axis_label))

@app.post("/axes/{axis_label}/stop")
async def stop_axis(axis_label: str, command: HTTPCommand) -> HTTPResponse:
    if command.command_arg == 1:
        if axis_label in motion_controller.axes.keys():
            #motion_controller.enqueue_machine_command(HTTPRobotCommand(command_type=ExternalCommandType.STOP, axis_label=axis_label))
            motion_controller.enqueue_machine_command(
                create_robot_command(axis_label=axis_label, command_type=ExternalCommandType.STOP))
            return HTTPResponse(success_string="Enqueued stop command for axis {}".format(axis_label))
        else:
            return HTTPResponse(error_string="Unknown axis {} requested for stopping".format(axis_label))


@app.post("/axes/{axis_label}/motion/absolute")
async def position_axis(axis_label: str, command: HTTPAxisPositioningCommand):
    if axis_label not in motion_controller.axes.keys():
        return HTTPResponse(error_string="Unknown axis {} requested for absolute positioning".format(axis_label))

    robot_command = HTTPRobotPositioningCommand()
    setattr(robot_command, axis_label, command)
    return handle_motion_command(robot_command)

@app.post("/axes/{axis_label}/motion/relative")
async def position_axis_relative(axis_label: str, command: HTTPAxisPositioningCommand):
    if axis_label not in motion_controller.axes_installed:
        return HTTPResponse(error_string="Axis {} is not installed in machine".format(axis_label))

    if command.target_position is None:
        return HTTPResponse(error_string="Must specify a non-null value for \"target_position\", not {}".format(str(command.target_position)))

    target_axis = motion_controller.axes[axis_label]
    command.target_position = command.target_position + target_axis.current_position

    robot_command = HTTPRobotPositioningCommand()
    setattr(robot_command, axis_label, command)
    return handle_motion_command(robot_command)

@app.post("/cell/{component_name}/{subcomponent_name}")
async def set_table_clamp(component_name: str, subcomponent_name: str, command: HTTPCommand):   
    controller_response = motion_controller.enqueue_machine_command(
                RobotCommand(command_type=ExternalCommandType.SET_CELL_SUBCOMPONENT_STATE,
                             cell_component=component_name,
                             cell_subcomponent=subcomponent_name,
                             state=command.subcomponent_state))
    if not controller_response.success:
        return BooleanHTTPResponse(error=True, response=controller_response.response_string)
    return BooleanHTTPResponse(response='Set table state to {}'.format(str(command.subcomponent_state)))


@app.post("/machine/gripper")
async def set_gripper(command: HTTPCommand):
    if command.gripper_state is not None and (command.gripper_state == 'open' or command.gripper_state == 'close'):
        if 'gripper_open' not in motion_controller.io_point_configuration['output'].keys():
            return HTTPResponse(error_string="Point function {} is not installed".format('gripper_open'))
        if 'gripper_close' not in motion_controller.io_point_configuration['output'].keys():
            return HTTPResponse(error_string="Point function {} is not installed".format('gripper_close'))

        for point, command_flag in zip(['gripper_open', 'gripper_close'], [bool(command.gripper_state == 'open'), not bool(command.gripper_state == 'open')]):
            controller_response = motion_controller.enqueue_machine_command(
                RobotCommand(command_type=ExternalCommandType.SET_IO_POINT,
                             point_function=point,
                             point_state=command_flag))
            if not controller_response.success:
                return HTTPResponse(error_string=controller_response.response_string)

        return HTTPResponse(success_string='Set gripper state to {}'.format(str(command.gripper_state)))

    else:
        return HTTPResponse(error_string='Must specify \"open\" or \"close\" for \"gripper_state\"')

@app.post("/machine/state")
async def set_machine_state(command: HTTPCommand) -> HTTPResponse:
    if command.state == 'manual':
        response = await motion_controller.manual_mode()
        #return HTTPResponse(error_string="Unknown state command")
    elif command.state == 'stop':
        response = await motion_controller.stop_mode()
        #return HTTPResponse(error_string="Unknown state command")
    elif command.state == 'sequence':
        response = await motion_controller.sequence_mode()
    elif command.state == 'abort':
        response = await motion_controller.abort()
        #return HTTPResponse(error_string="Unknown state command")
    else:
        return HTTPResponse(error_string="Unknown state command")

    if not response.success:
        return HTTPResponse(error_string="Unable to set state to " + command.state + ". Motion controller returned {}".format(response.response_string))
    else:
        return HTTPResponse(success_string=response.response_string)

# def validated_endpoint(handler, **kwargs) -> HTTPResponse:
#     if 'x' not in motion_controller.axes_installed:
#         return HTTPResponse(error_string="Axis {} is not installed")
#
#     controller_response = handler(kwargs)
#
#     if controller_response.success:
#         return HTTPResponse(success_string=controller_response.response_string)
#     else:
#         return HTTPResponse(error_string=controller_response.response_string)


@app.post("/axes/{axis_label}/io/output/{point_number}")
def set_axis_output_io_point(axis_label: str, point_number: int, command: HTTPCommand) -> HTTPResponse:
    if axis_label not in motion_controller.axes_installed:
        return HTTPResponse(error_string="Axis {} is not installed")

    target_axis = motion_controller.axes[axis_label]
    if point_number > target_axis.number_of_outputs or point_number < 0:
        return HTTPResponse(error_string="Invalid IO point number {}".format(point_number))

    controller_response = motion_controller.enqueue_machine_command(
        RobotCommand(command_type=ExternalCommandType.SET_IO_POINT,
                     target_axis_label=axis_label,
                     point_address=point_number,
                     point_state=bool(command.point_state)))

    if controller_response.success:
        return HTTPResponse(success_string=controller_response.response_string)
    else:
        return HTTPResponse(error_string=controller_response.response_string)

@app.post("/io/output/{point_function}")
async def set_output_io_point(point_function: str, command: HTTPCommand) -> HTTPResponse:
    if point_function not in motion_controller.io_point_configuration['output'].keys():
        return HTTPResponse(error_string="Point function {} is not installed".format(point_function))

    if type(command.point_state) is not int or (command.point_state != 0 and command.point_state != 1):
        return HTTPResponse(error_string="point_state must be either 0 or 1, not \"{}\"".format(str(command.point_state)))

    if motion_controller.io_point_configuration['output'][point_function]['bound_axis_label'] not in motion_controller.axes_installed:
        return HTTPResponse(
            error_string="IO point {} is bound to axis {}, which is not installed".format(point_function,
                                                                                          motion_controller.io_point_configuration[
                                                                                              'output'][point_function][
                                                                                              'bound_axis_label']))

    controller_response = motion_controller.enqueue_machine_command(
        RobotCommand(command_type=ExternalCommandType.SET_IO_POINT,
                     point_function=point_function,
                     point_state=bool(command.point_state)))

    if controller_response.success:
        return HTTPResponse(success_string=controller_response.response_string)
    else:
        return HTTPResponse(error_string=controller_response.response_string)

@app.post("/queue/sequence", response_model=BooleanHTTPResponse, response_model_exclude_unset=True)
async def enqueue_machine_sequence(command: HTTPCommand) -> BooleanHTTPResponse:
    if command.sequence_type not in ['pick', 'place']:
        return BooleanHTTPResponse(error=True, response="Must specify either \'pick\' or \'place\' for key \'sequence_type\'")

    if command.sequence_type == 'pick':
        sequence = MachineSequence(sequence_type=SequenceTypes.PICK)
    elif command.sequence_type == 'place':
        sequence = MachineSequence(sequence_type=SequenceTypes.PLACE)

    for _, field in command.__fields__.items():
        field_value = getattr(command, field.name)
        if field.type_ == HTTPAxisCommand and field_value is not None and field_value.target_position is not None:
            try:
                setattr(sequence, motion_controller.lookup_axis_function(field.name) + "_end", field_value.target_position)
            except KeyError:
                return BooleanHTTPResponse(error=True, response="{} axis not installed, cannot use for sequencing".format(field.name))

    # TODO catch case where target_position is not provided for any axis

    controller_response = motion_controller.enqueue_machine_sequence(sequence)
    return BooleanHTTPResponse(error=controller_response.success, response=controller_response.response_string)

@app.post("/queue/job", response_model=BooleanHTTPResponse, response_model_exclude_unset=True)
async def enqueue_machine_job(job: HTTPCommand) -> BooleanHTTPResponse:
    if job.origin == '' or job.destination == '':
        return BooleanHTTPResponse(error=True, response="Must supply valid origin and destination addresses as strings")

    if job.origin not in motion_controller.rack_locations.keys() and job.origin not in motion_controller.machine_locations.keys():
        return BooleanHTTPResponse(error=True, response="Unknown job origin {}".format(str(job.origin)))

    if job.destination not in motion_controller.rack_locations.keys() and job.destination not in motion_controller.machine_locations.keys():
        return BooleanHTTPResponse(error=True, response="Unknown job destination {}".format(str(job.destination)))

    #Determine if this is coming from a machine or coming from rack
    origin_locations = {}
    if job.origin in motion_controller.rack_locations.keys():
        for axis_function in motion_controller.axis_function_map.keys():
            origin_locations[axis_function + '_end'] = motion_controller.rack_locations[job.origin][axis_function]
    else:
        for axis_function in motion_controller.axis_function_map.keys():
            origin_locations[axis_function + '_end'] = motion_controller.machine_locations[job.origin][axis_function]

    destination_locations = {}
    if job.destination in motion_controller.rack_locations.keys():
        for axis_function in motion_controller.axis_function_map.keys():
            destination_locations[axis_function + '_end'] = motion_controller.rack_locations[job.destination][axis_function]
    else:
        for axis_function in motion_controller.axis_function_map.keys():
            destination_locations[axis_function + '_end'] = motion_controller.machine_locations[job.destination][axis_function]

    pick_controller_response = motion_controller.enqueue_machine_sequence(MachineSequence(sequence_type=SequenceTypes.PICK, target_location=job.origin, **origin_locations))
    place_controller_response = motion_controller.enqueue_machine_sequence(MachineSequence(sequence_type=SequenceTypes.PLACE, target_location=job.destination, **destination_locations))

    if pick_controller_response.success and place_controller_response.success:
        return BooleanHTTPResponse(response="Enqueued job with responses: {}, {}".format(pick_controller_response.response_string, place_controller_response.response_string))
    elif pick_controller_response.success:
        return BooleanHTTPResponse(error=True, response="Failed to enqueue place sequence")
    else:
        return BooleanHTTPResponse(error=True, response="Failed to enqueue pick sequence")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        try:
            try:
                tx_data = {}
                for _, axis in motion_controller.axes.items():
                    key = axis.axis_label + '_position'
                    tx_data[key] = axis.current_position

                await websocket.send_text(json.dumps(tx_data))
            except ConnectionClosedOK:
                print("websocket connection closed gracefully")
                break
            except ConnectionClosedError:
                print("websocket connection closed forcefully")
                break
            except Exception as e:
                a=5
                break
            
            await asyncio.sleep(0.5)

        except Exception as e:
            print('error: ', e)
            break

def setup_logger():
    if not os.path.exists(LOG_FILE_DIRECTORY):
        os.mkdir(LOG_FILE_DIRECTORY)

    for file in [f for f in os.listdir(LOG_FILE_DIRECTORY) if os.path.isfile(os.path.join(LOG_FILE_DIRECTORY, f))]:
        os.remove(os.path.join(LOG_FILE_DIRECTORY, file))


def motion_only(record) -> bool:
    if 'motion' in record['extra'].keys() and record['extra']['motion'] == True:
        return True
    return False

if __name__ == "__main__":
    time.time()

    setup_logger()

    logger.add(sys.stdout, format="{time} {level} {message}", level='INFO')
    #logger.add("logs/3r_motion_log_{time}.csv", format="{time:YYYY-MM-dd HH:mm:ss.SSS}, {message}", rotation="5 MB", retention="0.5h", enqueue=True, filter=motion_only, level='TRACE')
    logger.add(os.path.join(LOG_FILE_DIRECTORY, "3r_motion_log_{time}.csv"), format="{message}", rotation="5 MB",
               retention="0.5h", enqueue=True, filter=motion_only, level='TRACE')
    logger.add(os.path.join(LOG_FILE_DIRECTORY, '3r_error_log_{time}.log'), format='{time} {message}', rotation="5 MB", level='ERROR')

    mes_config = load_MES_config("config/mes_config.yaml")
    zmqContext = zmq.Context()
    status_socket = zmqContext.socket(zmq.PUB)
    status_socket.bind(f"tcp://*:{mes_config['communication']['status_socket']}")

    command_socket = zmqContext.socket(zmq.REP)
    command_socket.bind(f"tcp://*:{mes_config['communication']['command_socket']}")

    try:
        motion_controller = MotionController(configuration_file_name='robot_config.yaml',
                                             #machine_specific_configuration_file_path='config/machine_specific_config.yaml',
                                             calibration_table_name='calibration_table.yaml',
                                             #data_pub_socket=motion_data_socket,
                                             motion_data_stream_address=(mes_config['communication']['motion_data_stream_server_address'], mes_config['communication']['motion_data_stream_port'])
                                             )

        motion_controller.start()
    except Exception as e:
        logger.error(e.args[0])

    uvicorn.run(app, host="0.0.0.0", port=mes_config['communication']['fastapi_socket'])