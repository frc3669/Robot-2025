import cmath, commands2
from wpilib import SmartDashboard, DigitalInput, DriverStation, interfaces
from phoenix6 import hardware, controls, configs, signals, StatusCode
import constants

class CoralHandler(commands2.Subsystem):
    def __init__(self, controller: interfaces.GenericHID, cmd_controller: commands2.button.CommandGenericHID):
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
        # initialize stuff
        self.initCommandBindings()
        self.initMotorConfigs()
        self.initMovingAvg()
        self.initMiniKrakens()

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

    def setEjectCoralSpeed(self):
        if self.getCoralAngle() > 60:
            self.scoring_motor.set_control(controls.DutyCycleOut(0.25))
        elif self.getHeightReached(5):
            self.scoring_motor.set_control(controls.DutyCycleOut(-0.5))
        else:
            self.scoring_motor.set_control(controls.DutyCycleOut(-0.25))

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
            commands2.InstantCommand(lambda: self.setCoralAngle(15), self),
            commands2.WaitUntilCommand(lambda: self.getCoralAngleReached(15)),
            commands2.InstantCommand(lambda: self.setHeightAndAngles(height, coral_angle, algae_angle), self),
            commands2.WaitUntilCommand(lambda: self.getHeightReached(height)
                                       and self.getCoralAngleReached(coral_angle)
                                       and self.getAlgaeAngleReached(algae_angle))
        )
    
    def goL4Command(self) -> commands2.Command:
        return self.setHeightAndAnglesCommand(44, 89, 0)
    
    def goL3Command(self) -> commands2.Command:
        return self.setHeightAndAnglesCommand(6, 150, 0)
    
    def goL2Command(self) -> commands2.Command:
        return self.setHeightAndAnglesCommand(5, 0, 0)
    
    def goL1Command(self) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.InstantCommand(lambda: self.setIntakeSpeed(-0.18), self),
            commands2.WaitCommand(1),
            commands2.InstantCommand(lambda: self.brakeIntake(), self)
        )
    
    def ejectCoral(self) -> commands2.Command:
        return commands2.cmd.sequence(
            commands2.InstantCommand(lambda: self.setEjectCoralSpeed(), self),
            commands2.WaitCommand(1),
            commands2.InstantCommand(lambda: self.brakeIntake(), self)
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
            commands2.WaitUntilCommand(lambda: self.controller.getRawButton(8)),
            commands2.InstantCommand(lambda: self.setAlgaeIntakeSpeed(-0.4), self),
            commands2.WaitCommand(0.75),
            commands2.InstantCommand(lambda: self.brakeAlgaeIntake(), self)
        )

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
    
    def getCoralAngle(self) -> float:
        return (self.coralInitialAngle-self.coral_angle_motor.get_position().value_as_double)*360/constants.angle_gear_ratio

    def getHeight(self) -> float:
        return -self.elevator_motor.get_position().value_as_double/constants.elevator_in_to_rotations

    def teleopPeriodic(self):
        pass

    def periodic(self):
        if DriverStation.isTeleopEnabled():
            self.teleopPeriodic()
        SmartDashboard.putNumber("skew", self.skew)
        SmartDashboard.putNumber("distance", self.average_distance)
        SmartDashboard.putNumber("left_dist", self.left_distance)
        SmartDashboard.putNumber("right_dist", self.right_distance)
        SmartDashboard.putBoolean("coral detected", not self.intakeSensor.get())
        SmartDashboard.putBoolean("algae digital", self.algaeIntakeSensor.get())
        self.updateRangeAverages()

    def register(self):
        return super().register()
    
    def initCommandBindings(self):
        self.feederStationTrigger = self.cmd_controller.button(9).onTrue(self.intakeCommand())
        self.homeTrigger = self.cmd_controller.button(17).onTrue(self.homeCommand())
        self.algaeHomeTrigger = self.cmd_controller.button(6).onTrue(self.homeCommand())
        self.l1_coral_trigger = self.cmd_controller.button(16).onTrue(self.goL1Command())
        self.l2_coral_trigger = self.cmd_controller.button(15).onTrue(self.goL2Command())
        self.l3_coral_trigger = self.cmd_controller.button(14).onTrue(self.goL3Command())
        self.l4_coral_trigger = self.cmd_controller.button(13).onTrue(self.goL4Command())
        self.scoreCoralTrigger = self.cmd_controller.button(10).onTrue(self.ejectCoral())
        self.intakeAlgaeTrigger = self.cmd_controller.button(5).onTrue(self.intakeAlgaeCommand())
        self.intakeL3_5Trigger = self.cmd_controller.button(3).onTrue(self.intakeL3_5())
        self.intakeL2_5Trigger = self.cmd_controller.button(4).onTrue(self.intakeL2_5())
        self.scoreBargeTrigger = self.cmd_controller.button(1).onTrue(self.scoreBargeCommand())
        self.scoreProcessorTrigger = self.cmd_controller.button(2).onTrue(self.scoreProcessorCommand())

    def initMotorConfigs(self):
        # coral angle motor configuration 
        coral_angle_cfg = configs.TalonFXConfiguration()
        coral_angle_cfg.slot0.k_p = 40
        coral_angle_cfg.slot0.k_s = 4
        coral_angle_cfg.torque_current.peak_forward_torque_current = 50
        coral_angle_cfg.torque_current.peak_reverse_torque_current = -50
        coral_angle_cfg.motion_magic.motion_magic_acceleration = 80
        coral_angle_cfg.motion_magic.motion_magic_cruise_velocity = 80
        coral_angle_cfg.motion_magic.motion_magic_jerk = 300
        # algae angle motor configuration 
        algae_angle_cfg = configs.TalonFXConfiguration()
        algae_angle_cfg.slot0.k_p = 30
        algae_angle_cfg.slot0.k_s = 4
        algae_angle_cfg.torque_current.peak_forward_torque_current = 30
        algae_angle_cfg.torque_current.peak_reverse_torque_current = -30
        algae_angle_cfg.motion_magic.motion_magic_acceleration = 20
        algae_angle_cfg.motion_magic.motion_magic_cruise_velocity = 40
        algae_angle_cfg.motion_magic.motion_magic_jerk = 200
        # elevotor motor configs
        elevator_cfg = configs.TalonFXConfiguration()
        elevator_cfg.slot0.k_p = 30
        elevator_cfg.slot0.k_s = 5
        elevator_cfg.slot0.k_g = -10
        elevator_cfg.torque_current.peak_forward_torque_current = 70
        elevator_cfg.torque_current.peak_reverse_torque_current = -70
        elevator_cfg.motion_magic.motion_magic_acceleration = 50
        elevator_cfg.motion_magic.motion_magic_cruise_velocity = 100
        elevator_cfg.motion_magic.motion_magic_jerk = 300
        # scoring motor configs
        scoring_cfg = configs.TalonFXSConfiguration()
        scoring_cfg.commutation.motor_arrangement = signals.MotorArrangementValue.MINION_JST
        scoring_cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # scoring motor configs
        algae_scoring_cfg = configs.TalonFXSConfiguration()
        algae_scoring_cfg.commutation.motor_arrangement = signals.MotorArrangementValue.MINION_JST
        algae_scoring_cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # dictionary to store the list of motors, configs, config status, and names
        configs_dict = {
            "motors": [self.elevator_motor, self.coral_angle_motor, self.algae_angle_motor, self.scoring_motor, self.algae_scoring_motor],
            "configs": [elevator_cfg, coral_angle_cfg, algae_angle_cfg, scoring_cfg, algae_scoring_cfg],
            "status": [StatusCode.STATUS_CODE_NOT_INITIALIZED]*5,
            "names": ["elevator motor", "coral angle motor", "algae angle motor", "coral scoring motor", "algae scoring motor"]
        }
        # Retry config apply up to 5 times, report if failure
        for _ in range(5):
            for i in range(len(configs_dict["motors"])):
                if not configs_dict["status"][i].is_ok():
                    configs_dict["status"][i] = configs_dict["motors"][i].configurator.apply(configs_dict["configs"][i])
            if all([status.is_ok() for status in configs_dict["status"]]):
                break
        for i in range(len(configs_dict["motors"])):
            if not configs_dict["status"][i].is_ok():
                print(f"Could not apply {configs_dict["names"][i]} configs, error code: {configs_dict["status"][i].name}")

    def initMovingAvg(self):
        self._i_range = 0
        self._past_left_range_values = [0 for _ in range(constants.run_ave_max_index)]
        self._past_right_range_values = [0 for _ in range(constants.run_ave_max_index)]
        self.run_ave_difference = 0
        self.left_distance = 0
        self.right_distance = 0
        self.skew = 0
        self.average_distance = 0

    def initMiniKrakens(self):
        self.coralInitialAngle = self.coral_angle_motor.get_position().value_as_double
        self.algaeInitialAngle = self.algae_angle_motor.get_position().value_as_double

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