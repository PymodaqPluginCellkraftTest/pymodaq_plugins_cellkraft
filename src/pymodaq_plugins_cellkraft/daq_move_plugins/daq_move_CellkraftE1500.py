from typing import Union, List, Dict
from pymodaq_plugins_cellkraft import config
from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, DataActuatorType,\
    DataActuator  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter
from pymodaq_plugins_cellkraft.hardware.cellkraft.Eseries import CellKraftE1500Drivers
from pymodaq_utils.logger import set_logger, get_module_name

logger = set_logger(get_module_name(__file__))


class DAQ_Move_CellkraftE1500(DAQ_Move_base):
    """

    pymodaq Version 5.0.11
    pymodaq-data Version 5.0.23

    Limites thérorique a ne pas dépasser :
            - Flow : 2 g/min                                                                - modifiable     - move
           X- Pressure                                                                      - non modifiable - viewer0D
            - Steam Temperature : 180 °C                                                    - modifiable     - move
            - Tube Temperature : 200 °C                                                     - modifiable     - move
            - RH (Relative Humidity) : 80%                                                  - modifiable     - move

    - Modifiez le fichier : ...\site-packages\pymodaq\control_modules\daq_move.py
    - A la ligne  615, modifiez :
    - self.ui.set_unit_as_suffix(self.get_unit_to_display(unit))
    - en
    - self.ui.set_unit_as_suffix(unit)
    - Cela empeche la conversion des unites rentrée dans _controller_units

    - Si des "m" sont devant les unitées dans le dashboard, copier cela dans le config_pymodaq.toml:
    [actuator]
    epsilon_default = 1
    polling_interval_ms = 100
    polling_timeout_s = 20  # s
    refresh_timeout_ms = 500  # ms
    siprefix = false
    siprefix_even_without_units = false
    display_units = true

    """
    is_multiaxes = False

    _axis_names: Union[List[str], Dict[str, str]] = ['Flow', 'Steam_Temperature', 'Tube_Temperature', 'RH', 'Pressure']
    _controller_units: Union[str, List[str]] = ['g/min', '°C', '°C', '%', 'bar']
    _epsilon: Union[float, List[float]] = [0.1, 0.1, 0.1, 1, 1]

    data_actuator_type = DataActuatorType.DataActuator

    params = [{'title': 'Device:', 'name': 'device', 'type': 'str', 'value': 'Cellkraft E1500 Series',
               'readonly': True},
              {'title': 'Host:', 'name': 'host', 'type': 'str', 'value': config('Cellkraft', 'DEVICE01', 'host')},
              {'title': 'Write Limit:', 'name': 'limit', 'type': 'str', 'value': 'Nothing'},
              {'title': 'Info:', 'name': 'info', 'type': 'str', 'value': 'Nothing', 'readonly': True},
              ] + comon_parameters_fun(is_multiaxes=is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    # Information sur chaque actuateurs
    desc = {'Flow': "Lecture en 0.1 près, Ecriture en 0.1 près\n"
                    "En cas de dépassement de cette limite, "
                    "le surplus sera effacé (exemple : 1.25 va ressortir en 1.2)",

            'Steam_Temperature': "Lecture en 0.1 près, Ecriture en 1 près\n"
                    "En cas de dépassement de cette limite, "
                    "le surplus sera effacé (exemple : 100.1 va ressortir en 100)",

            'Tube_Temperature': "Lecture en 0.1 près, Ecriture en 0.1 près\n"
                    "En cas de dépassement de cette limite, "
                    "le surplus sera effacé (exemple : 100.25 va ressortir en 100.2)",

            'RH': "Lecture en 0.1 près, Ecriture en 0.1 près\n"
                    "En cas de dépassement de cette limite, "
                    "le surplus sera effacé (exemple : 95.25 va ressortir en 95.2)",
            'Pressure': "None"
            }

    # Limite de valeur pour chaque actuateur, (modifiable dans le dashboard via le show settings)
    lim = {'Flow': 2.5,
           'Steam_Temperature': 160,
           'Tube_Temperature': 160,
           'RH': 100,
           'Pressure': 0
           }

    current_axes: str

    def ini_attributes(self):
        self.controller: CellKraftE1500Drivers
        self.current_axes = self.settings.child('multiaxes', 'axis').value()
        self.change_param()
        pass

    def change_param(self):

        if self.current_axes == 'Flow':
            self.settings.child('info').setValue(self.desc['Flow'])
            self.settings.child('limit').setValue(self.lim['Flow'])

        elif self.current_axes == 'Steam_Temperature':
            self.settings.child('info').setValue(self.desc['Steam_Temperature'])
            self.settings.child('limit').setValue(self.lim['Steam_Temperature'])

        elif self.current_axes == 'Tube_Temperature':
            self.settings.child('info').setValue(self.desc['Tube_Temperature'])
            self.settings.child('limit').setValue(self.lim['Tube_Temperature'])

        elif self.current_axes == 'RH':
            self.settings.child('info').setValue(self.desc['RH'])
            self.settings.child('limit').setValue(self.lim['RH'])

        elif self.current_axes == 'Pressure':
            self.settings.child('info').setValue(self.desc['Pressure'])
            self.settings.child('limit').setValue(self.lim['Pressure'])

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        if self.current_axes == 'Flow':
            flow = DataActuator(data=self.controller.Get_Flow())
            flow = self.get_position_with_scaling(flow)
            return flow

        elif self.current_axes == 'RH':
            air_h = DataActuator(data=self.controller.Get_Air_H())
            air_h = self.get_position_with_scaling(air_h)
            return air_h

        elif self.current_axes == 'Steam_Temperature':
            steam_t = DataActuator(data=self.controller.Get_Steam_T())
            steam_t = self.get_position_with_scaling(steam_t)
            return steam_t

        elif self.current_axes == 'Tube_Temperature':
            tube_t = DataActuator(data=self.controller.Get_Tube_T())
            tube_t = self.get_position_with_scaling(tube_t)
            return tube_t

        elif self.current_axes == 'Pressure':
            pressure = DataActuator(data=self.controller.Get_Pressure())
            pressure = self.get_position_with_scaling(pressure)
            return pressure

        else:
            self.emit_status(ThreadCommand('Update_Status',
                                           ['WARNING - No Axis Selected, self.current_axes can be None']))

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
                self.current_axes = 'Flow'

            elif param.value() == 'Steam_Temperature':
                self.axis_unit = self._controller_units[1]
                self.settings.child('units').value = self._controller_units[1]
                self.current_axes = 'Steam_Temperature'

            elif param.value() == 'Tube_Temperature':
                self.axis_unit = self._controller_units[2]
                self.settings.child('units').value = self._controller_units[2]
                self.current_axes = 'Tube_Temperature'

            elif param.value() == 'RH':
                self.axis_unit = self._controller_units[3]
                self.settings.child('units').value = self._controller_units[3]
                self.current_axes = 'RH'

            elif param.value() == 'Pressure':
                self.axis_unit = self._controller_units[4]
                self.settings.child('units').value = self._controller_units[4]
                self.current_axes = 'Pressure'
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

        # Debug Master/Slave
        # if controller is None:
        #     self.emit_status(ThreadCommand('Update_Status', [f'Controller is None - {self.current_axes} is Master']))
        # else:
        #   self.emit_status(ThreadCommand('Update_Status', [f'Controller is not None - {self.current_axes} is Slave']))

        if self.is_master:  # Master Case : controller == None
            controller = CellKraftE1500Drivers(self.settings['host'])  # Create control
            self.controller = controller
            initialized = self.controller.init_hardware()  # Init connection
            info = "Initialized in Master"

        else:  # Slave Case : controller != None
            self.controller = self.ini_stage_init(slave_controller=controller)
            initialized = self.controller.instr.connected
            info = "Initialized in Slave"
        self.controller.PumpSetMode("auto")
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
        Used by move_rel() and move_abs()
        Parameters
        ----------
        value: (float) value of the target positioning
        """
        limit = self.settings.child('limit').opts['value']
        if self.current_axes == 'Flow':
            if not value.value() < float(limit):
                self.emit_status(ThreadCommand('Update_Status', [f'WARNING - Flow have to be < {limit}']))
            else:
                val = self.controller.SP_Flow(value.value())*0.1
                self.emit_status(ThreadCommand('Update_Status', [f'Flow set to {val}']))

        elif self.current_axes == 'RH':
            if not value.value() < float(limit):
                self.emit_status(ThreadCommand('Update_Status', [f'WARNING - RH have to be < {limit}']))
            else:
                val = self.controller.RH(value.value())*0.1
                self.emit_status(ThreadCommand('Update_Status', [f'RH set to {val}']))

        elif self.current_axes == 'Steam_Temperature':
            if not value.value() < float(limit):
                self.emit_status(ThreadCommand('Update_Status', [f'WARNING - Steam_Temp have to be < {limit}']))
            else:
                val = self.controller.SP_SteamT(int(value.value()))
                self.emit_status(ThreadCommand('Update_Status', [f'Steam_Temp set to {val}']))

        elif self.current_axes == 'Tube_Temperature':
            if not value.value() < float(limit):
                self.emit_status(ThreadCommand('Update_Status', [f'WARNING - Tube_Temp have to be < {limit}']))
            else:
                val = self.controller.SP_Tube_Temp(int(value.value()))*0.1
                self.emit_status(ThreadCommand('Update_Status', [f'Tube_Temp set to {val}']))

        elif self.current_axes == 'Pressure':
            self.emit_status(ThreadCommand('Update_Status', ["WARNING - Can't modify the pressure"]))

        else:
            self.emit_status(ThreadCommand('Update_Status',
                                           ['WARNING - Nothing moved - Problem with current_axes variable in'
                                            'daq_move_CELLkraftE1500.py - WARNING']))

    def move_home(self, value: Union[float, DataActuator]):
        """
        Set the value to 0 / can change this value later
        """
        pass

    def stop_motion(self):
        """
        Stop the actuator and emits move_done signal
        Pump is the only one that can be stopped, the other are just values that we change
        """
        pass
