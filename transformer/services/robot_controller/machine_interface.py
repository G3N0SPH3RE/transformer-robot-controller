from typing import ClassVar, List, Dict, Any, Tuple
from dataclasses import dataclass, field
from loguru import logger
from .festo_interface import FestoManifold
#from .motion_control import MotionController

@dataclass
class Door:
    parent_machine: object

    open_confirmation_input_io_point: Dict = None

    def opened(self) -> bool:
        for _, input in self.parent_machine.cell_controller.axes[self.open_confirmation_input_io_point['bound_axis_label']].inputs.items():
            if input['point_address'] == self.open_confirmation_input_io_point['point_address']:
                return input['point_state']

        #return self.parent_machine.cell_controller.axes[self.open_confirmation_io_point['bound_axis_label']].inputs

@dataclass
class Table:
    #control_festo_manifold: bool = False
    parent_machine: object

    control_type: str = ''
    festo_manifold_address: str = ''
    nominal_supply_pressure_bar: float = 0
    allowable_pressure_delta_bar: float = 0

    compax3_clamp_io_point: Dict = None
    compax3_unclamp_io_point: Dict = None
    
    table_clamp_manifold: ClassVar[FestoManifold] = None
    #valid_states: StaticVar[List[str]] = ['clamp', 'unclamp']
    valid_states = ['clamp', 'unclamp']

    def __post_init__(self):
        if self.control_type == 'festo':
            self.table_clamp_manifold = FestoManifold(self.festo_manifold_address)

            main_pressure = (self.table_clamp_manifold.get_analog_input_values(0)/1000) * 14.5038

            # Capture schunk clamp station pressure and convert from mbar to psi
            # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
            schunk_pressure = (self.table_clamp_manifold.get_analog_input_values(1)/1000) * 14.5038

            # Capture Gressell Riser clamp station pressure and convert from mbar to psi
            # Station clamped = ~0 psi, Unclamped ~=MainPressureInPSI
            gressel_pressure = (self.table_clamp_manifold.get_analog_input_values(1)/1000) * 14.5038

            logger.info("Machine {} Festo manifold main pressure: {}".format(self.parent_machine.name, main_pressure))
            logger.info("Machine {} Schunk clamp pressure: {}".format(self.parent_machine.name, schunk_pressure))
            logger.info("Machine {} GR clamp pressure: {}".format(self.parent_machine.name, gressel_pressure))
        else:
            if self.compax3_clamp_io_point is not None:
                logger.info(f"Machine {self.parent_machine.name} table clamp set up on {self.compax3_clamp_io_point['bound_axis_label']} axis output {self.compax3_clamp_io_point['point_address']}")
            if self.compax3_unclamp_io_point is not None:
                logger.info(f"Machine {self.parent_machine.name} table unclamp set up on {self.compax3_unclamp_io_point['bound_axis_label']} axis output {self.compax3_unclamp_io_point['point_address']}")
    
    def set_state(self, state: str):
        self.set_table_clamp(state == 'clamp')

    def set_table_clamp(self, clamped: bool) -> bool:
        if self.control_type == 'festo':
            self.table_clamp_manifold.change_valve_state(FestoManifold.schunkclamp, FestoManifold.clamp if clamped else FestoManifold.unclamp)
        elif self.control_type == 'compax3':
            if self.compax3_clamp_io_point is not None:
                self.parent_machine.cell_controller.axes[self.compax3_clamp_io_point['bound_axis_label']].set_IO_point(port=self.parent_machine.cell_controller.port,
                                                        io_point_address=self.compax3_unclamp_io_point['point_address'],
                                                        io_point_value=not clamped)

            if self.compax3_unclamp_io_point is not None:
                self.parent_machine.cell_controller.axes[self.compax3_unclamp_io_point['bound_axis_label']].set_IO_point(port=self.parent_machine.cell_controller.port,
                                                        io_point_address=self.compax3_unclamp_io_point['point_address'],
                                                        io_point_value=not clamped)

    def check_table_unclamped(self) -> bool:
        unclamped = True
        if self.control_type == 'festo':
            supply_pressure = self.table_clamp_manifold.get_analog_input_values(FestoManifold.main_pressure_input)/1000.0
            unclamped &= (supply_pressure >= (self.nominal_supply_pressure_bar - self.allowable_pressure_delta_bar))
            if unclamped == False:
                logger.warning(f"Machine {self.parent_machine.name} detected low supply pressure of {supply_pressure} bar on Schunk clamp when checking clamped state")

            unclamped &= (self.table_clamp_manifold.get_analog_input_values(FestoManifold.schunk_pressure_input)/1000.0 >= (self.nominal_supply_pressure_bar - self.allowable_pressure_delta_bar))
        elif self.control_type == 'compax3':
            pass
        
        return unclamped
            # self.axes[self.compax3_clamp_io_point_address[0]].set_IO_point(port=self.port,
            #                                          io_point_address=compax3_clamp_io_point_address[1],
            #                                          io_point_value=clamped)
            
            # if 'table_unclamp' in self.parent_machine.cell_controller.io_point_configuration['output'].keys():
            #     unclamp_bound_axis_label = self.parent_machine.cell_controller.io_point_configuration['output']['table_unclamp']['bound_axis_label']
            #     unclamp_point_address = self.parent_machine.cell_controller.io_point_configuration['output']['table_unclamp']['point_address']
            #     self.axes[unclamp_bound_axis_label].set_IO_point(port=self.port,
            #                                          io_point_address=unclamp_point_address,
            #                                          io_point_value=not clamped)

            # if 'table_clamp' in self.parent_machine.cell_controller.io_point_configuration['output'].keys():
            #     clamp_bound_axis_label = self.parent_machine.cell_controller.io_point_configuration['output']['table_clamp']['bound_axis_label']
            #     clamp_point_address = self.parent_machine.cell_controller.io_point_configuration['output']['table_clamp']['point_address']
            #     self.axes[clamp_bound_axis_label].set_IO_point(port=self.port,
            #                                          io_point_address=clamp_point_address,
            #                                          io_point_value=clamped)


@dataclass
class MachineTool:
    name: str
    cell_controller: object
    
    #Setup parameters
    configuration: Dict[str, Any]

    # Position calibration
    reach: float = 0.0
    traverse: float = 0.0
    vertical: float = 0.0
    swing: float = 120.0
    clearance_height: float = 0.0
    reach_safe_position: float = 0.0

    table: ClassVar[Table] = None
    door: ClassVar[Door] = None

    def __post_init__(self):
        if 'table' in self.configuration.keys():
            self.table = Table(parent_machine=self, **self.configuration['table'])
        
        if 'door' in self.configuration.keys():
            self.door = Door(parent_machine=self, **self.configuration['door'])