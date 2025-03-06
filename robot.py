import wpilib, commands2
from wpilib import DataLogManager, XboxController, SmartDashboard
import wpilib.drive
from subsystems.swerve import Swerve, SwerveModule, Trajectory
from subsystems.coralHandler import CoralHandler
from subsystems.climb import Climb
# from phoenix6 import hardware, controls, configs, StatusCode

class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.controller = wpilib.Joystick(0)
        self.keypad = wpilib.Joystick(1)
        self.coralHandler = CoralHandler(self.keypad)
        self.climb = Climb()
        
        # Make sure we start at 0
        # Run once when the robot first turns on
        # self is where you can place class variables. They will be
        #  available in all class functions
        Swerve.add_module(SwerveModule(1, 1, 1))
        Swerve.add_module(SwerveModule(2, -1, 1))
        Swerve.add_module(SwerveModule(3, -1, -1))
        Swerve.add_module(SwerveModule(4, 1, -1))


    
    def teleopInit(self):
        DataLogManager.start()
        self.coralHandler.goHome()

        

    def teleopPeriodic(self):
        # Runs once every 50ms
        Swerve.driveTeleop(self.controller, self.controller.getRawButton(0), self.coralHandler.skew)
        #self.keypad.getRawButtonPressed())
        #self.elevatorMotor.set_control(controls.DutyCycleOut(self.controller.getRawAxis(0)))
        if self.keypad.getRawButton(18):
            self.climb.extend()
        elif self.keypad.getRawButton(19):
            self.climb.retract()
        else:
            self.climb.brake()

        self.coralHandler.updateRangeAverages()
        SmartDashboard.putNumber("Skew", self.coralHandler.skew)
        SmartDashboard.putNumber("Average_distance", self.coralHandler.average_distance)
        SmartDashboard.putNumber('elevator_pos', self.coralHandler.elevator_motor.get_position().value_as_double)
        

    
    def autonomousInit(self) -> None:
        commands2.cmd.sequence(
            Swerve.followTrajectory(Trajectory("Test.traj")),
            Swerve.followTrajectory(Trajectory("Test.traj"))).schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous"""
        stage = 0
        wait = 0

        if stage == 0:
            # do something
            pass
            # until
            if wait > 100:
                wait = 0
                stage += 1
        elif stage == 1:
            pass

if __name__ == "__main__":
    wpilib.run(Robot)