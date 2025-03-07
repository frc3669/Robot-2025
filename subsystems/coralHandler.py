import cmath, commands2
from wpilib import Timer, SmartDashboard, DataLogManager, DigitalInput, DriverStation, interfaces
from phoenix6 import hardware, controls, configs, signals, StatusCode
import utils.mathFunctions as mf
import constants

class CoralHandler(commands2.Subsystem):
    def __init__(self, controller: interfaces.GenericHID, cmd_controller: commands2.button.CommandGenericHID):
        super().__init__()
        self.canr0 = hardware.CANrange(9, "CTREdevices")
        self.canr1 = hardware.CANrange(10, "CTREdevices")
        self.intakeSensor = DigitalInput(0)
        self.elevator_motor = hardware.TalonFX(41, "CTREdevices")
        self.scoring_motor = hardware.TalonFXS(61, "CTREdevices")
        self.angle_motor = hardware.TalonFX(51, "CTREdevices")
        self.velocity_ctrl = controls.VelocityTorqueCurrentFOC(0)
        self.position_torque = controls.PositionTorqueCurrentFOC(0)
        self.position_ctrl = controls.PositionDutyCycle(0)
        self.controller = controller
        self.cmd_controller = cmd_controller
        # set up event triggers
        self.feederStationTrigger = self.cmd_controller.button(9)
        self.feederStationTrigger.onTrue(self.intakeCommand())
        self.homeTrigger = self.cmd_controller.button(17)
        self.homeTrigger.onTrue(self.homeCommand())
        self.l1_coral_trigger = self.cmd_controller.button(16)
        self.l1_coral_trigger.onTrue(self.goL1Command())
        self.l2_coral_trigger = self.cmd_controller.button(15)
        self.l2_coral_trigger.onTrue(self.goL2Command())
        self.l3_coral_trigger = self.cmd_controller.button(14)
        self.l3_coral_trigger.onTrue(self.goL3Command())
        self.l4_coral_trigger = self.cmd_controller.button(13)
        self.l4_coral_trigger.onTrue(self.goL4Command())
        # angle motor configuration 
        angle_cfg = configs.TalonFXConfiguration()
        angle_cfg.slot0.k_p = 70
        angle_cfg.slot0.k_d = 5
        angle_cfg.torque_current.peak_forward_torque_current = 20
        angle_cfg.torque_current.peak_reverse_torque_current = -40
        # elevotor motor configs
        elevator_cfg = configs.TalonFXConfiguration()
        elevator_cfg.slot0.k_p = 25
        elevator_cfg.slot0.k_d = 2
        elevator_cfg.slot0.k_g = -15
        elevator_cfg.torque_current.peak_forward_torque_current = 15
        elevator_cfg.torque_current.peak_reverse_torque_current = -45
        # scoring motor configs
        scoring_cfg = configs.TalonFXSConfiguration()
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

    
    def setHeight(self, height):
        rotations = -height*constants.elevator_in_to_rotations
        self.elevator_motor.set_control(self.position_torque.with_position(rotations))

    def setAngle(self, angle):
        rotations = -angle/360*constants.coral_angle_gear_ratio
        self.angle_motor.set_control(self.position_torque.with_position(rotations))

    def setHeightAndAngle(self, height, angle):
        self.setHeight(height)
        self.setAngle(angle)

    def brakeIntake(self):
        self.scoring_motor.set_control(controls.NeutralOut())

    def intakeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAngleCommand(11, 22),
            commands2.FunctionalCommand(
                lambda: self.setIntakeSpeed(0.25),
                self.doNothing,
                self.doNothing,
                lambda: not self.intakeSensor.get(),
                self
            ),
            commands2.FunctionalCommand(
                lambda: self.scoring_motor.set(-0.1),
                self.doNothing,
                lambda x: self.brakeIntake(),
                lambda: self.intakeSensor.get(),
                self
            ),
            commands2.FunctionalCommand(
                lambda: self.scoring_motor.set(0.1),
                self.doNothing,
                lambda x: self.brakeIntake(),
                lambda: not self.intakeSensor.get(),
                self
            )
        )
    
    def homeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.FunctionalCommand(
                lambda: self.setHeightAndAngle(5, 10),
                self.doNothing,
                self.doNothing,
                lambda: self.getHeightReached(5) and self.getAngleReached(10),
                self
            ),
            commands2.FunctionalCommand(
                lambda: self.setHeight(0),
                self.doNothing,
                lambda x: self.setAngle(-5),
                lambda: self.getHeightReached(0),
                self
            )
        )
    
    def setHeightAndAngleCommand(self, height, angle) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.FunctionalCommand(
                lambda: self.setAngle(10),
                self.doNothing,
                self.doNothing,
                lambda: self.getAngleReached(10),
                self
            ),
            commands2.FunctionalCommand(
                lambda: self.setHeight(5),
                self.doNothing,
                self.doNothing,
                lambda: self.getHeightReached(5),
                self
            ),
            commands2.FunctionalCommand(
                lambda: self.setHeightAndAngle(height, angle),
                self.doNothing,
                self.doNothing,
                lambda: self.getHeightReached(height) and self.getAngleReached(angle),
                self
            )
        )
    
    def goL4Command(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAngleCommand(44, 83),
            commands2.WaitUntilCommand(lambda: self.controller.getRawButton(10)),
            commands2.FunctionalCommand(
                lambda: self.setIntakeSpeed(0.25),
                self.doNothing,
                lambda x: self.brakeIntake(),
                self.intakeSensor.get,
                self
            )
        )
    
    def goL3Command(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAngleCommand(6, 145),
            commands2.WaitUntilCommand(lambda: self.controller.getRawButton(10)),
            commands2.FunctionalCommand(
                lambda: self.setIntakeSpeed(0.25),
                self.doNothing,
                lambda x: self.brakeIntake(),
                self.intakeSensor.get,
                self
            )
        )
    
    def goL2Command(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAngleCommand(5, -10),
            commands2.WaitUntilCommand(lambda: self.controller.getRawButton(10)),
            commands2.InstantCommand(
                lambda: self.setIntakeSpeed(-0.25),
                self
            ),
            commands2.WaitCommand(1),
            commands2.InstantCommand(
                lambda: self.brakeIntake()
            )
        )
    
    def goL1Command(self) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.InstantCommand(
                lambda: self.setIntakeSpeed(-0.18),
                self
            ),
            commands2.WaitCommand(1),
            commands2.InstantCommand(
                lambda: self.brakeIntake()
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
        SmartDashboard.putNumber("skew", self.skew)
        SmartDashboard.putNumber("distance", self.average_distance)
        SmartDashboard.putNumber("left_dist", self.left_distance)
        SmartDashboard.putNumber("right_dist", self.right_distance)
    
    def periodic(self):
        super().periodic()
        if DriverStation.isTeleopEnabled():
            self.teleopPeriodic()

    # function that does nothing
    def doNothing(self, x = False):
        pass

    def getHeightReached(self, position) -> bool:
        return abs(-self.elevator_motor.get_position().value_as_double/constants.elevator_in_to_rotations - position) < 0.5
    
    def getAngleReached(self, angle) -> bool:
        current_angle = -self.angle_motor.get_position().value_as_double*360/constants.coral_angle_gear_ratio
        return abs(current_angle - angle) < 5