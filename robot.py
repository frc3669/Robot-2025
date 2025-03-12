import wpilib, commands2, choreo
from wpilib import DataLogManager, SmartDashboard
import wpilib.interfaces
from subsystems.swerve import Swerve, SwerveModule
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
        self.leftPath1 = choreo.load_swerve_trajectory("Left Path 1")
        self.leftPath2 = choreo.load_swerve_trajectory("Left Path 2")
        self.centerPath1 = choreo.load_swerve_trajectory("Center Path 1")
        self.odometryTestPath = choreo.load_swerve_trajectory("Odometry Test")
        self.scoreRightCmd = commands2.cmd.race(commands2.cmd.sequence(Swerve.driveRightToPole(), self.coralHandler.ejectCoral()), commands2.WaitCommand(3))
        self.scoreRightTrigger = self.command_keypad.button(12).onTrue(self.scoreRightCmd)
        self.scoreLeftCmd = commands2.cmd.race(commands2.cmd.sequence(Swerve.driveLeftToPole(), self.coralHandler.ejectCoral()), commands2.WaitCommand(3))
        self.scoreLeftTrigger = self.command_keypad.button(11).onTrue(self.scoreLeftCmd)
    
    def teleopInit(self):
        DataLogManager.start()
        self.coralHandler.homeCommand().schedule()
        self.coralHandler.brakeIntake()

    def teleopPeriodic(self):
        # Runs once every 20ms

        if not self.scoreRightCmd.isScheduled() and not self.scoreLeftCmd.isScheduled():
            Swerve.driveTeleop(self.controller, self.controller.getRawButton(1), self.coralHandler.skew)
    
    def autonomousInit(self) -> None:
        self.coralHandler.setHeightAndAngles(0, 0, 0)
        self.coralHandler.brakeIntake()
        initial_pose = self.centerPath1.get_initial_pose()
        commands2.cmd.sequence(
            Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
            Swerve.followTrajectory(self.odometryTestPath)
        ).schedule()

    def autonomousPeriodic(self):
        pass

    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

if __name__ == "__main__":
    wpilib.run(Robot)