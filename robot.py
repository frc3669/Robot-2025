import wpilib, commands2, choreo
from wpilib import DataLogManager, XboxController, SmartDashboard
import wpilib.interfaces
from subsystems.swerve import Swerve, SwerveModule, Trajectory
from subsystems.coralHandler import CoralHandler
from subsystems.climb import Climb

class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.controller = wpilib.interfaces.GenericHID(0)
        self.keypad = wpilib.interfaces.GenericHID(1)
        self.command_keypad = commands2.button.CommandGenericHID(1)
        self.coralHandler = CoralHandler(self.keypad, self.command_keypad)
        self.climb = Climb(self.keypad)
        Swerve.add_module(SwerveModule(1, 1, 1))
        Swerve.add_module(SwerveModule(2, -1, 1))
        Swerve.add_module(SwerveModule(3, -1, -1))
        Swerve.add_module(SwerveModule(4, 1, -1))
        self.trajectories = [
            choreo.load_swerve_trajectory("Center to center-right L4"),
            choreo.load_swerve_trajectory("center-right to feeder station right")
        ]
        self.initial_pose = self.trajectories[0].get_initial_pose()
        Swerve.position = complex(self.initial_pose.x, self.initial_pose.y)
        Swerve.startingAngle = self.initial_pose.rotation().radians()
    
    def teleopInit(self):
        DataLogManager.start()
        self.coralHandler.setHeightAndAngle(0, -5)

    def teleopPeriodic(self):
        # Runs once every 20ms
        Swerve.driveTeleop(self.controller, self.controller.getRawButton(1), self.coralHandler.skew)
        self.coralHandler.updateRangeAverages()
        
    
    def autonomousInit(self) -> None:
        commands2.cmd.sequence(Swerve.followTrajectory(trajectory) for trajectory in self.trajectories).schedule()

    def autonomousPeriodic(self):
        pass

    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

if __name__ == "__main__":
    wpilib.run(Robot)