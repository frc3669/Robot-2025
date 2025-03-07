import cmath, commands2
from wpilib import SmartDashboard, DriverStation, interfaces
from phoenix6 import hardware, controls, configs, StatusCode, signals
import constants

class Climb(commands2.Subsystem):
    def __init__(self, controller: interfaces.GenericHID):
        super().__init__()
        self.climbMotor = hardware.TalonFX(43, "CTREdevices")
        # create configuration 
        cfg = configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.climbMotor.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        self.controller = controller
    
    def retract(self):
        self.climbMotor.set_control(controls.DutyCycleOut(1))
    
    def extend(self):
        self.climbMotor.set_control(controls.DutyCycleOut(-1))
    
    def brake(self):
        self.climbMotor.set_control(controls.NeutralOut())

    def teleopPeriodic(self):
        if self.controller.getRawButton(18):
            self.extend()
        elif self.controller.getRawButton(19):
            self.retract()
        else:
            self.brake()
    
    def periodic(self):
        super().periodic()
        if DriverStation.isTeleopEnabled():
            self.teleopPeriodic()