from typing import Union, List, Dict

from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main, DataActuatorType,\
    DataActuator  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_cellkraft.hardware.cellkraft.Eseries import CellKraftE1500Drivers, Eseries_Config
from pymodaq_utils.logger import set_logger, get_module_name

logger = set_logger(get_module_name(__file__))


class DAQ_Move_CellkraftE1500(DAQ_Move_base):
    """

    Si des "m" sont devant les unitées dans le dashboard, copier cela dans le config_pymodaq.toml:
    [actuator]
    epsilon_default = 1
    polling_interval_ms = 100
    polling_timeout_s = 20  # s
    refresh_timeout_ms = 500  # ms
    siprefix = false
    siprefix_even_without_units = false
    display_units = true

    """
    is_multiaxes = True

    _axis_names: Union[List[str], Dict[str, str]] = ['Flow', 'Pressure', 'Steam_Temperature', 'Tube_Temperature', 'RH']
    _controller_units: Union[str, List[str]] = ['g/min', "bar", '°C', '°C', '%']
    _epsilon: Union[float, List[float]] = [0.1, 0.1, 0.1, 0.1, 1]

    data_actuator_type = DataActuatorType.DataActuator

    params = [{'title': 'Device:', 'name': 'device', 'type': 'str', 'value': 'Cellkraft E1500 Series',
               'readonly': True},
              {'title': 'Host:', 'name': 'host', 'type': 'str', 'value': 'cet-cc01-gen01.insa-lyon.fr'},
              {'title': 'Comments:', 'name': 'comment', 'type': 'text', 'value': ''},
              ] + comon_parameters_fun(is_multiaxes=is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    current_axes: str

    def ini_attributes(self):
        self.controller: CellKraftE1500Drivers
        self.current_axes = self.settings.child('multiaxes','axis').value()
        self.controller.PumpSetMode("auto")
        # declare here attributes you want/need to init with a default value
        pass

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        if self.current_axes == 'flow':
            flow = DataActuator(data=self.controller.Get_Flow())
            flow = self.get_position_with_scaling(flow)
            return flow

        elif self.current_axes == 'pressure':
            pressure = DataActuator(data=self.controller.Get_Pressure())
            pressure = self.get_position_with_scaling(pressure)
            return pressure

        elif self.current_axes == 'rh':
            air_H = DataActuator(data=self.controller.Get_Air_H)
            air_H = self.get_position_with_scaling(air_H)
            return air_H

        elif self.current_axes == 'Steam_temperature':
            Steam_T = DataActuator(data=self.controller.Get_Steam_T)
            steam_T = self.get_position_with_scaling(Steam_T)
            return steam_T

        elif self.current_axes == 'Tube_temperature':
            tube_T = DataActuator(data=self.controller.Get_Tube_T)
            tube_T = self.get_position_with_scaling(tube_T)
            return tube_T

        else:
            flow = DataActuator(data=self.controller.Get_Flow())
            flow = self.get_position_with_scaling(flow)
            return flow


    def user_condition_to_reach_target(self) -> bool:
        """ Implement a condition for exiting the polling mechanism and specifying that the
        target value has been reached

       Returns
        -------
        bool: if True, PyMoDAQ considers the target value has been reached
        """
        #  either delete this method if the usual polling is fine with you, but if need you can
        #  add here some other condition to be fullfilled either a completely new one or
        #  using or/and operations between the epsilon_bool and some other custom booleans
        #  for a usage example see DAQ_Move_brushlessMotor from the Thorlabs plugin
        return True

    def close(self):
        """Terminate the communication protocol"""
        self.controller.close()

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == 'axis':
            if param.value() == 'Flow':
                self.axis_unit = self._controller_units[0]
                self.settings.child('units').value = self._controller_units[0]
                self.current_axes = 'flow'

            elif param.value() == 'Pressure':
                self.axis_unit = self._controller_units[1]
                self.settings.child('units').value = self._controller_units[1]
                self.current_axes = 'pressure'

            elif param.value() == 'Steam_Temperature':
                self.axis_unit = self._controller_units[2]
                self.settings.child('units').value = self._controller_units[2]
                self.current_axes = 'Steam_temperature'

            elif param.value() == 'Tube_Temperature':
                self.axis_unit = self._controller_units[2]
                self.settings.child('units').value = self._controller_units[2]
                self.current_axes = 'Steam_temperature'

            elif param.value() == 'RH':
                self.axis_unit = self._controller_units[3]
                self.settings.child('units').value = self._controller_units[3]
                self.current_axes = 'rh'
        pass


    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        self.ini_stage_init(slave_controller=controller)  # will be useful when controller is slave

        if self.is_master:  # is needed when controller is master
            self.controller = CellKraftE1500Drivers(self.settings['host'])  # arguments for instantiation!)
            #  enter here whatever is needed for your controller initialization and eventual
            #  opening of the communication channel

        self.controller.init_hardware()
        info = "Initialized"
        initialized = True
        # initialized = self.controller.init_hardware()
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  # if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one

        self.move_value(value)

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)

        self.move_value(value)

    def move_value(self, value):
        """ Move the actuator to the target actuator value defined by value
        / Used by move_rel and move_abs \
        Parameters
        ----------
        value: (float) value of the target positioning
        """
        if self.current_axes == 'Flow':
            if not value.value() < 5:
                self.emit_status(ThreadCommand('Update_Status', ['WARNING - Flow have to be < 5']))
                self.controller.SP_Flow(1)
            else:
                self.controller.SP_Flow(int(value.value()))
            self.emit_status(ThreadCommand('Update_Status', ['Flow set to {}'.format(value)]))

        elif self.current_axes == 'Pressure':
            if not value.value() < 110:  # Peut etre modifier la limite si besoin #
                self.emit_status(ThreadCommand('Update_Status', ['WARNING - Pressure have to be < 110']))
                self.controller.Pump(100)
            else:
                self.controller.Pump(int(value.value()))
            self.emit_status(ThreadCommand('Update_Status', ['Pressure set to {}'.format(value)]))

        elif self.current_axes == 'RH':
            if not value.value() < 110:  # Peut etre modifier la limite si besoin #
                self.emit_status(ThreadCommand('Update_Status', ['WARNING - RH have to be < 110']))
                self.controller.RH(105)
            else:
                self.controller.RH(int(value.value()))
            self.emit_status(ThreadCommand('Update_Status', ['RH set to {}'.format(value)]))

        elif self.current_axes == 'Steam_Temperature':
            if not value.value() < 30:  # Peut etre modifier la limite si besoin #
                self.emit_status(ThreadCommand('Update_Status', ['WARNING - Steam_Temp have to be < 30']))
                self.controller.SP_SteamT(10)
            else:
                self.controller.SP_SteamT(int(value.value()))
            self.emit_status(ThreadCommand('Update_Status', ['Steam_Temp set to {}'.format(value)]))

        elif self.current_axes == 'Tube_Temperature':
            if not value.value() < 30:  # Peut etre modifier la limite si besoin #
                self.emit_status(ThreadCommand('Update_Status', ['WARNING - Tube_Temp have to be < 30']))
                self.controller.SP_Tube_Temp(10)
            else:
                self.controller.SP_Tube_Temp(int(value.value()))
            self.emit_status(ThreadCommand('Update_Status', ['Tube_Temp set to {}'.format(value)]))

        else:
            self.emit_status(ThreadCommand('Update_Status', ['WARNING - Nothing moved - Problem with current_axes variable in daq_move_CELLkraftE1500.py - WARNING']))

    def move_home(self):
        """
        Set the value to 0 / can change this value later
        """

        self.move_value(0)

    def stop_motion(self):
        """
        Stop the actuator and emits move_done signal
        Pump is the only one that can be stopped, the other are just values that we change

        Notes :
        La fonction stop en l'etat ne sert a rien car elle renvoie vers SP_Flow(0),
        mais mieux vaut la garder pour de futur modifications
        / fonction stop a supprimer si aucune modif faite
        """
        if self.current_axes == 'flow':
            self.controller.stop()
            self.emit_status(ThreadCommand('Update_Status', ['Flow Stopped']))


if __name__ == '__main__':
    main(__file__)
