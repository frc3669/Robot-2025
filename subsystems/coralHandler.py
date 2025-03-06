import cmath, commands2
from wpilib import Timer, SmartDashboard, DataLogManager, DigitalInput, DriverStation, Joystick
from phoenix6 import hardware, controls, configs, signals, StatusCode
import utils.mathFunctions as mf
import constants

class CoralHandler(commands2.Subsystem):
    def __init__(self, controller: Joystick):
        super().__init__()
        self.canr0 = hardware.CANrange(9, "CTREdevices")
        self.canr1 = hardware.CANrange(10, "CTREdevices")
        self.intakeSensor = DigitalInput(0)
        self.elevator_motor = hardware.TalonFX(41, "CTREdevices")
        self.scoring_motor = hardware.TalonFXS(61, "CTREdevices")
        self.angle_motor = hardware.TalonFX(51, "CTREdevices")
        self.velocity_ctrl = controls.VelocityTorqueCurrentFOC(0)
        self.position_ctrl = controls.PositionDutyCycle(0)
        self.controller = controller
        self.feeder_station_button = commands2.button.JoystickButton(self.controller, 9)
        self.feeder_station_button.onTrue(self.intakeCommand())
        # self.home_button = commands2.button.JoystickButton(self.controller, 17)
        # self.feeder_station_button.onTrue(self.homeCommand())
        self.l1_coral_button = None
        # create configuration 
        angle_cfg = configs.TalonFXConfiguration()
        angle_cfg.slot0.k_p = 0.5
        angle_cfg.voltage.peak_forward_voltage = 1.5
        angle_cfg.voltage.peak_reverse_voltage = -1.5
        angle_cfg.current_limits.stator_current_limit_enable = True
        angle_cfg.current_limits.supply_current_limit = 20
        angle_cfg.current_limits.supply_current_lower_limit = 10
        # angle_cfg.torque_current.peak_forward_torque_current
        elevator_cfg = configs.TalonFXConfiguration()
        elevator_cfg.slot0.k_p = 0.2
        elevator_cfg.voltage.peak_forward_voltage = 1.5
        elevator_cfg.voltage.peak_reverse_voltage = -3
        elevator_cfg.current_limits.stator_current_limit_enable = True
        elevator_cfg.current_limits.supply_current_limit = 20
        elevator_cfg.current_limits.supply_current_lower_limit = 10
        scoring_cfg = configs.TalonFXSConfiguration()
        scoring_cfg.slot0.k_p = 5
        scoring_cfg.commutation.motor_arrangement = signals.MotorArrangementValue.MINION_JST
        scoring_cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE

        # Retry config apply up to 5 times, report if failure
        status1, status2, status3 = [StatusCode.STATUS_CODE_NOT_INITIALIZED]*3
        for _ in range(0, 5):
            if not status1.is_ok():
                status1 = self.elevator_motor.configurator.apply(elevator_cfg)
            if not status2.is_ok():
                status2 = self.angle_motor.configurator.apply(angle_cfg)
            if not status3.is_ok():
                status2 = self.scoring_motor.configurator.apply(scoring_cfg)
            if status1.is_ok() and status2.is_ok() and status3.is_ok():
                break
        if not status1.is_ok():
            print(f"Could not apply coral elevator configs, error code: {status1.name}")
        if not status2.is_ok():
            print(f"Could not apply coral angle motor configs, error code: {status2.name}")
        if not status3.is_ok():
            print(f"Could not apply scoring motor configs, error code: {status3.name}")

        self.initMovAvg()
        self.left_distance = 0
        self.right_distance = 0
        self.skew = 0
        self.average_distance = 0

    def initMovAvg(self):
        self._i_range = 0
        self._past_left_range_values = [0 for _ in range(constants.run_ave_max_index)]
        self._past_right_range_values = [0 for _ in range(constants.run_ave_max_index)]
        self.run_ave_difference = 0

    def setIntakeSpeed(self, speed):
        self.scoring_motor.set(speed)

    
    def setElevatorHeight(self, height):
        rotations = -height*constants.coral_elevator_in_to_rotations
        self.elevator_motor.set_control(self.position_ctrl.with_position(rotations))

    def setAngle(self, angle):
        rotations = -angle/360*constants.coral_angle_gear_ratio
        self.angle_motor.set_control(self.position_ctrl.with_position(rotations))

    def intake(self):
        self.setIntakeSpeed(0.25)
        self.setElevatorHeight(20)
        self.setAngle(30)
        print("intaking")
    
    def goHome(self, input: bool = False):
        self.scoring_motor.set_control(controls.NeutralOut())
        self.setElevatorHeight(0)
        self.setAngle(0)
        print("going home")

    def lowerElevator(self, input: bool = False):
        self.setIntakeSpeed(0)
        self.setElevatorHeight(0)
        self.setAngle(20)
        print("lowering elevator")

    #def placeCoralL4(self):
        #self.setElevatorHeight(constants.L4_height)
        #self.setIntakeSpeed(constants.release_intake)

   # def placeCoralL3(self):
       # self.setElevatorHeight(constants.L3_height)
       # self.setIntakeSpeed(constants.release_intake)

    #def placeCoralL2(self):
        #self.setElevatorHeight(constants.L2_height)
        #self.setIntakeSpeed(constants.release_intake)

   # def placeCoralL1(self):
       # self.setElevatorHeight(constants.L1_height)
       # #somehow rotate coral horizontally
        #self.setIntakeSpeed(constants.release_intake)


    def intakeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.FunctionalCommand(
                self.intake,
                self.doNothing,
                self.doNothing,
                lambda: not self.intakeSensor.get(),
                CoralHandler
            ),
            commands2.FunctionalCommand(
                lambda: self.scoring_motor.set(-0.1),
                self.doNothing,
                lambda: self.scoring_motor.set_control(controls.NeutralOut()),
                lambda: self.intakeSensor.get(),
                CoralHandler
            )
        )
    
    def homeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.FunctionalCommand(
                self.lowerElevator,
                self.doNothing,
                self.doNothing,
                lambda: abs(self.elevator_motor.get_position().value_as_double - 0) < 0.5,
                CoralHandler
            ),
            commands2.FunctionalCommand(
                self.goHome,
                self.doNothing,
                self.doNothing,
                lambda: not self.intakeSensor.get(),
                CoralHandler
            )
        )

    def updateRangeAverages(self):
        """
        Print left and right rangefinder values
            print values as 1s running averages
        """
        self._past_left_range_values[self._i_range] = self.canr0.get_distance().value_as_double
        self.left_distance = sum(self._past_left_range_values)/constants.run_ave_max_index
        
        self._past_right_range_values[self._i_range] = self.canr1.get_distance().value_as_double
        self.right_distance = sum(self._past_right_range_values)/constants.run_ave_max_index

        self.skew = self.left_distance - self.right_distance
        self.average_distance = (self.left_distance + self.right_distance) / 2
        # at end of function, increment index. Loop at max index
        self._i_range += 1
        if self._i_range == constants.run_ave_max_index:
            self._i_range = 0

    def teleopPeriodic(self):
        pass
    
    def periodic(self):
        super().periodic()
        if DriverStation.isTeleopEnabled():
            self.teleopPeriodic()

    # function that does nothing
    def doNothing(self, x = False):
        pass