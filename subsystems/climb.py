import cmath, commands2
from wpilib import Timer, SmartDashboard, DataLogManager, DigitalInput
from phoenix6 import hardware, controls, configs, StatusCode, signals
import constants

class Climb(commands2.Subsystem):
    def __init__(self):
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
    
    def retract(self):
        self.climbMotor.set_control(controls.DutyCycleOut(1))
    
    def extend(self):
        self.climbMotor.set_control(controls.DutyCycleOut(-1))
    
    def brake(self):
        self.climbMotor.set_control(controls.NeutralOut())