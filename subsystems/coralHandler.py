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
        self.algaeIntakeSensor = DigitalInput(2)
        self.elevator_motor = hardware.TalonFX(41, "CTREdevices")
        self.scoring_motor = hardware.TalonFXS(61, "CTREdevices")
        self.algae_scoring_motor = hardware.TalonFXS(62, "CTREdevices")
        self.coral_angle_motor = hardware.TalonFX(51, "CTREdevices")
        self.algae_angle_motor = hardware.TalonFX(52, "CTREdevices")
        self.velocity_ctrl = controls.VelocityTorqueCurrentFOC(0)
        self.position_torque = controls.PositionTorqueCurrentFOC(0)
        self.position_velocity = controls.MotionMagicTorqueCurrentFOC(0)
        self.position_ctrl = controls.PositionDutyCycle(0)
        self.controller = controller
        self.cmd_controller = cmd_controller
        # set up button event triggers
        self.feederStationTrigger = self.cmd_controller.button(9)
        self.feederStationTrigger.onTrue(self.intakeCommand())
        self.homeTrigger = self.cmd_controller.button(17)
        self.homeTrigger.onTrue(self.homeCommand())
        self.algaeHomeTrigger = self.cmd_controller.button(6)
        self.algaeHomeTrigger.onTrue(self.homeCommand())
        self.l1_coral_trigger = self.cmd_controller.button(16)
        self.l1_coral_trigger.onTrue(self.goL1Command())
        self.l2_coral_trigger = self.cmd_controller.button(15)
        self.l2_coral_trigger.onTrue(self.goL2Command())
        self.l3_coral_trigger = self.cmd_controller.button(14)
        self.l3_coral_trigger.onTrue(self.goL3Command())
        self.l4_coral_trigger = self.cmd_controller.button(13)
        self.l4_coral_trigger.onTrue(self.goL4Command())
        self.intakeAlgaeTrigger = self.cmd_controller.button(5)
        self.intakeAlgaeTrigger.onTrue(self.intakeAlgaeCommand())
        self.intakeL3_5Trigger = self.cmd_controller.button(3)
        self.intakeL3_5Trigger.onTrue(self.intakeL3_5())
        self.intakeL2_5Trigger = self.cmd_controller.button(4)
        self.intakeL2_5Trigger.onTrue(self.intakeL2_5())
        self.scoreBargeTrigger = self.cmd_controller.button(1)
        self.scoreBargeTrigger.onTrue(self.scoreBargeCommand())
        self.scoreProcessorTrigger = self.cmd_controller.button(2)
        self.scoreProcessorTrigger.onTrue(self.scoreProcessorCommand())
        # coral angle motor configuration 
        coral_angle_cfg = configs.TalonFXConfiguration()
        coral_angle_cfg.slot0.k_p = 40
        coral_angle_cfg.torque_current.peak_forward_torque_current = 50
        coral_angle_cfg.torque_current.peak_reverse_torque_current = -50
        coral_angle_cfg.motion_magic.motion_magic_acceleration = 50
        coral_angle_cfg.motion_magic.motion_magic_cruise_velocity = 80
        coral_angle_cfg.motion_magic.motion_magic_jerk = 200
        # algae angle motor configuration 
        algae_angle_cfg = configs.TalonFXConfiguration()
        algae_angle_cfg.slot0.k_p = 15
        algae_angle_cfg.torque_current.peak_forward_torque_current = 30
        algae_angle_cfg.torque_current.peak_reverse_torque_current = -30
        algae_angle_cfg.motion_magic.motion_magic_acceleration = 20
        algae_angle_cfg.motion_magic.motion_magic_cruise_velocity = 40
        algae_angle_cfg.motion_magic.motion_magic_jerk = 200
        # elevotor motor configs
        elevator_cfg = configs.TalonFXConfiguration()
        elevator_cfg.slot0.k_p = 20
        elevator_cfg.slot0.k_g = -10
        elevator_cfg.torque_current.peak_forward_torque_current = 70
        elevator_cfg.torque_current.peak_reverse_torque_current = -70
        elevator_cfg.motion_magic.motion_magic_acceleration = 50
        elevator_cfg.motion_magic.motion_magic_cruise_velocity = 100
        elevator_cfg.motion_magic.motion_magic_jerk = 200
        # scoring motor configs
        scoring_cfg = configs.TalonFXSConfiguration()
        scoring_cfg.commutation.motor_arrangement = signals.MotorArrangementValue.MINION_JST
        scoring_cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # scoring motor configs
        algae_scoring_cfg = configs.TalonFXSConfiguration()
        algae_scoring_cfg.commutation.motor_arrangement = signals.MotorArrangementValue.MINION_JST
        algae_scoring_cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # Retry config apply up to 5 times, report if failure
        status1, status2, status3, status4, status5 = [StatusCode.STATUS_CODE_NOT_INITIALIZED]*5
        for _ in range(0, 5):
            if not status1.is_ok():
                status1 = self.elevator_motor.configurator.apply(elevator_cfg)
            if not status2.is_ok():
                status2 = self.coral_angle_motor.configurator.apply(coral_angle_cfg)
            if not status3.is_ok():
                status3 = self.scoring_motor.configurator.apply(scoring_cfg)
            if not status4.is_ok():
                status4 = self.algae_angle_motor.configurator.apply(algae_angle_cfg)
            if not status5.is_ok():
                status5 = self.algae_scoring_motor.configurator.apply(algae_scoring_cfg)
            if status1.is_ok() and status2.is_ok() and status3.is_ok() and status4.is_ok() and status5.is_ok():
                break
        if not status1.is_ok():
            print(f"Could not apply coral elevator configs, error code: {status1.name}")
        if not status2.is_ok():
            print(f"Could not apply coral angle motor configs, error code: {status2.name}")
        if not status3.is_ok():
            print(f"Could not apply scoring motor configs, error code: {status3.name}")
        if not status4.is_ok():
            print(f"Could not apply algae angle motor configs, error code: {status4.name}")
        if not status5.is_ok():
            print(f"Could not apply algae scoring motor configs, error code: {status5.name}")
        # set up moving average
        self._i_range = 0
        self._past_left_range_values = [0 for _ in range(constants.run_ave_max_index)]
        self._past_right_range_values = [0 for _ in range(constants.run_ave_max_index)]
        self.run_ave_difference = 0
        self.left_distance = 0
        self.right_distance = 0
        self.skew = 0
        self.average_distance = 0

    def intitialize(self):
        self.coralInitialAngle = self.coral_angle_motor.get_position().value_as_double
        self.algaeInitialAngle = self.algae_angle_motor.get_position().value_as_double

    def setIntakeSpeed(self, speed):
        self.scoring_motor.set(speed)

    def setAlgaeIntakeSpeed(self, speed):
        self.algae_scoring_motor.set(speed)
    
    def setHeight(self, height):
        rotations = -height*constants.elevator_in_to_rotations
        self.elevator_motor.set_control(self.position_velocity.with_position(rotations))

    def setCoralAngle(self, angle):
        rotations = self.coralInitialAngle - angle/360*constants.angle_gear_ratio
        self.coral_angle_motor.set_control(self.position_velocity.with_position(rotations))

    def setAlgaeAngle(self, angle):
        rotations = self.algaeInitialAngle + angle/360*constants.algae_angle_gear_ratio
        self.algae_angle_motor.set_control(self.position_velocity.with_position(rotations))

    def setHeightAndAngles(self, height, coral_angle, algae_angle):
        self.setHeight(height)
        self.setCoralAngle(coral_angle)
        self.setAlgaeAngle(algae_angle)

    def brakeIntake(self, x = False):
        self.scoring_motor.set_control(controls.NeutralOut())

    def brakeAlgaeIntake(self, x = False):
        self.algae_scoring_motor.set_control(controls.NeutralOut())

    def stopEverything(self):
        self.scoring_motor.set_control(controls.NeutralOut())
        self.algae_scoring_motor.set_control(controls.NeutralOut())

    def intakeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(11, 32, 0),
            commands2.InstantCommand(lambda: self.setIntakeSpeed(0.25), self),
            commands2.WaitUntilCommand(lambda: not self.intakeSensor.get()),
            commands2.InstantCommand(lambda: self.setIntakeSpeed(-0.1), self),
            commands2.WaitUntilCommand(lambda: self.intakeSensor.get()),
            commands2.InstantCommand(lambda: self.setIntakeSpeed(0.1), self),
            commands2.WaitUntilCommand(lambda: not self.intakeSensor.get()),
            commands2.InstantCommand(lambda: self.brakeIntake(), self)
        )

    def intakeAlgaeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(0, 0, 170),
            commands2.InstantCommand(lambda: self.setAlgaeIntakeSpeed(0.25), self),
            commands2.WaitUntilCommand(lambda: not self.algaeIntakeSensor.get()),
            commands2.InstantCommand(lambda: self.brakeAlgaeIntake(), self)
        )
    
    def intakeL2_5(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(15, 0, 110),
            commands2.InstantCommand(lambda: self.setAlgaeIntakeSpeed(0.25), self),
            commands2.WaitUntilCommand(lambda: not self.algaeIntakeSensor.get()),
            commands2.InstantCommand(lambda: self.brakeAlgaeIntake(), self)
        )
    
    def intakeL3_5(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(30, 0, 110),
            commands2.InstantCommand(lambda: self.setAlgaeIntakeSpeed(0.25), self),
            commands2.WaitUntilCommand(lambda: not self.algaeIntakeSensor.get()),
            commands2.InstantCommand(lambda: self.brakeAlgaeIntake(), self)
        )
    
    def homeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.InstantCommand(lambda: self.stopEverything(), self),
            commands2.InstantCommand(lambda: self.setHeightAndAngles(0, 15, 0), self),
            commands2.WaitUntilCommand(lambda: self.getHeightReached(0) and self.getAlgaeAngleReached(0) and self.getCoralAngleReached(15)),
            commands2.InstantCommand(lambda: self.setHeightAndAngles(0, 0, 0), self)
        )
         
    def setHeightAndAnglesCommand(self, height, coral_angle, algae_angle) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.FunctionalCommand(
                lambda: self.setCoralAngle(15),
                self.doNothing,
                self.doNothing,
                lambda: self.getCoralAngleReached(15),
                self
            ),
            commands2.FunctionalCommand(
                lambda: self.setHeightAndAngles(height, coral_angle, algae_angle),
                self.doNothing,
                self.doNothing,
                lambda: self.getHeightReached(height) and self.getCoralAngleReached(coral_angle) and self.getAlgaeAngleReached(algae_angle),
                self
            )
        )
    
    def goL4Command(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(44, 89, 0),
            commands2.WaitUntilCommand(lambda: self.controller.getRawButton(10)),
            commands2.FunctionalCommand(
                lambda: self.setIntakeSpeed(0.25),
                self.doNothing,
                lambda x: self.brakeIntake(),
                self.intakeSensor.get,
                self
            )
        )
    
    def goL4Auto(self) -> commands2.Command:
        return self.setHeightAndAnglesCommand(44, 93, 0)
    
    def goL3Command(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(6, 155, 0),
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
            self.setHeightAndAnglesCommand(5, 0, 0),
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
    
    def ejectCoral(self, isReversed: bool = False) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.InstantCommand(
                lambda: self.setIntakeSpeed(-0.25 if isReversed else 0.25),
                self
            ),
            commands2.WaitCommand(1),
            commands2.InstantCommand(
                lambda: self.brakeIntake()
            )
        )

    def scoreBargeCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(49, 0, 30),
            commands2.WaitUntilCommand(lambda: self.controller.getRawButton(8)),
            commands2.InstantCommand(lambda: self.setAlgaeIntakeSpeed(-0.4), self),
            commands2.WaitCommand(0.75),
            commands2.InstantCommand(lambda: self.brakeAlgaeIntake(), self)
        )

    def scoreProcessorCommand(self) -> commands2.Command:
        return commands2.cmd.sequence(
            self.setHeightAndAnglesCommand(0, 0, 140),
            commands2.WaitUntilCommand(lambda: self.controller.getRawButton(2)),
            commands2.InstantCommand(lambda: self.setAlgaeIntakeSpeed(-0.4), self),
            commands2.WaitCommand(0.75),
            commands2.InstantCommand(lambda: self.brakeAlgaeIntake(), self)
        )

    # function that does nothing
    def doNothing(self, x = False):
        pass

    def getHeightReached(self, position) -> bool:
        return abs(-self.elevator_motor.get_position().value_as_double/constants.elevator_in_to_rotations - position) < 0.5
    
    def getCoralAngleReached(self, angle) -> bool:
        current_angle = (self.coralInitialAngle-self.coral_angle_motor.get_position().value_as_double)*360/constants.angle_gear_ratio
        return abs(current_angle - angle) < 5
    
    def getAlgaeAngleReached(self, angle) -> bool:
        current_angle = (-self.algaeInitialAngle+self.algae_angle_motor.get_position().value_as_double)*360/constants.algae_angle_gear_ratio
        return abs(current_angle - angle) < 5
    
    def getAlgaeAngle(self) -> float:
        return (-self.algaeInitialAngle+self.algae_angle_motor.get_position().value_as_double)*360/constants.algae_angle_gear_ratio

    def getHeight(self) -> float:
        return -self.elevator_motor.get_position().value_as_double/constants.elevator_in_to_rotations

    def updateRangeAverages(self):
        """
        get left and right CANRange distances
        and update their running averages
        """
        # add new values
        self._past_left_range_values[self._i_range] = self.canr0.get_distance().value_as_double
        self._past_right_range_values[self._i_range] = self.canr1.get_distance().value_as_double
        # average
        self.left_distance = sum(self._past_left_range_values)/constants.run_ave_max_index
        self.right_distance = sum(self._past_right_range_values)/constants.run_ave_max_index
        # calculte skew and distance from reef
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
        SmartDashboard.putBoolean("coral detected", not self.intakeSensor.get())
        SmartDashboard.putBoolean("algae digital", self.algaeIntakeSensor.get())
        self.updateRangeAverages()
    
    def periodic(self):
        super().periodic()
        if DriverStation.isTeleopEnabled():
            self.teleopPeriodic()