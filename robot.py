import wpilib, commands2, choreo
from wpilib import SmartDashboard
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
        # add autonomous selector to Smartdashboard
        self.chooser = wpilib.SendableChooser()
        self.calibrationAuto = "Calibration"
        self.centerAuto = "Center"
        self.leftAuto = "Left"
        self.rightAuto = "Right"
        self.centerAutoAlgae = "Center Algae"
        self.leftAutoAlgae = "Left Algae"
        self.rightAutoAlgae = "Right Algae"
        self.chooser.setDefaultOption("Calibrate Odometry", self.calibrationAuto)
        self.chooser.addOption("Center", self.centerAuto)
        self.chooser.addOption("Left", self.leftAuto)
        self.chooser.addOption("Right", self.rightAuto)
        self.chooser.addOption("Center Algae", self.centerAutoAlgae)
        self.chooser.addOption("Left Algae", self.leftAutoAlgae)
        self.chooser.addOption("Right Algae", self.rightAutoAlgae)
        SmartDashboard.putData("Auto choices", self.chooser)
        # load Choreo paths
        self.leftPath1 = choreo.load_swerve_trajectory("Left Path 1")
        self.leftPath2 = choreo.load_swerve_trajectory("Left Path 2")
        self.centerPath1 = choreo.load_swerve_trajectory("Center Path 1")
        self.centerPath2 = choreo.load_swerve_trajectory("Center Path 2")
        self.rightPath1 = choreo.load_swerve_trajectory("Right Path 1")
        self.rightPath2 = choreo.load_swerve_trajectory("Right Path 2")
        self.odometryTestPath = choreo.load_swerve_trajectory("Odometry Test")
        self.centerAlgae1 = choreo.load_swerve_trajectory("Center Algae 1")
        self.centerAlgae2 = choreo.load_swerve_trajectory("Center Algae 2")
        self.leftAlgae1 = choreo.load_swerve_trajectory("Left Algae 1")
        self.leftAlgae2 = choreo.load_swerve_trajectory("Left Algae 2")
        self.rightAlgae1 = choreo.load_swerve_trajectory("Right Algae 1")
        self.rightAlgae2 = choreo.load_swerve_trajectory("Right Algae 2")
        # create commands for auto scoring
        self.scoreRightCmd = commands2.cmd.race(commands2.cmd.sequence(Swerve.driveRightToPole(), self.coralHandler.ejectCoral()), commands2.WaitCommand(3))
        self.scoreLeftCmd = commands2.cmd.race(commands2.cmd.sequence(Swerve.driveLeftToPole(), self.coralHandler.ejectCoral()), commands2.WaitCommand(3))
        self.scoreRightTrigger = self.command_keypad.button(12).onTrue(self.scoreRightCmd)
        self.scoreLeftTrigger = self.command_keypad.button(11).onTrue(self.scoreLeftCmd)
    
    def teleopInit(self):
        self.autoSelected = self.chooser.getSelected()
        if self.autoSelected == self.calibrationAuto or self.autoSelected == self.centerAuto or self.autoSelected == self.leftAuto or self.autoSelected == self.rightAuto:
            self.coralHandler.homeCommand().schedule()
        elif self.autoSelected == self.centerAutoAlgae:
            self.coralHandler.intakeL2_5().schedule()
        else:
            self.coralHandler.intakeL3_5().schedule()
        self.coralHandler.brakeIntake()

    def teleopPeriodic(self):
        if not self.scoreRightCmd.isScheduled() and not self.scoreLeftCmd.isScheduled():
            Swerve.driveTeleop(self.controller, self.controller.getRawButton(1), self.controller.getRawButton(2))
    
    def autonomousInit(self) -> None:
        self.autoSelected = self.chooser.getSelected()
        print("Auto selected: " + self.autoSelected)
        self.coralHandler.setHeightAndAngles(0, 0, 0)
        self.coralHandler.brakeIntake()
        match self.autoSelected:
            case self.calibrationAuto:
                initial_pose = self.odometryTestPath.get_initial_pose()
                commands2.cmd.sequence(
                    Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
                    Swerve.followTrajectory(self.odometryTestPath)
                ).schedule()
            case self.centerAuto:
                initial_pose = self.centerPath1.get_initial_pose()
                commands2.cmd.sequence(
                    Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
                    commands2.cmd.parallel(
                        Swerve.followTrajectory(self.centerPath1),
                        self.coralHandler.goL4Command()),
                    self.scoreRightCmd,
                    Swerve.resetPositionCmd(complex(self.centerPath2.get_initial_pose().x, self.centerPath2.get_initial_pose().y)),
                    Swerve.followTrajectory(self.centerPath2)
                ).schedule()
            case self.leftAuto:
                initial_pose = self.leftPath1.get_initial_pose()
                commands2.cmd.sequence(
                    Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
                    commands2.cmd.parallel(
                        Swerve.followTrajectory(self.leftPath1),
                        self.coralHandler.goL4Command()),
                    self.scoreLeftCmd,
                    Swerve.resetPositionCmd(complex(self.leftPath2.get_initial_pose().x, self.leftPath2.get_initial_pose().y)),
                    Swerve.followTrajectory(self.leftPath2)
                ).schedule()
            case self.rightAuto:
                initial_pose = self.rightPath1.get_initial_pose()
                commands2.cmd.sequence(
                    Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
                    commands2.cmd.parallel(
                        Swerve.followTrajectory(self.rightPath1),
                        self.coralHandler.goL4Command()),
                    self.scoreRightCmd,
                    Swerve.resetPositionCmd(complex(self.rightPath2.get_initial_pose().x, self.rightPath2.get_initial_pose().y)),
                    Swerve.followTrajectory(self.rightPath2)
                ).schedule()
            case self.centerAutoAlgae:
                initial_pose = self.centerPath1.get_initial_pose()
                commands2.cmd.sequence(
                    Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
                    commands2.cmd.parallel(
                        Swerve.followTrajectory(self.centerPath1),
                        self.coralHandler.goL4Command()),
                    self.scoreRightCmd,
                    Swerve.resetPositionCmd(complex(self.centerPath2.get_initial_pose().x, self.centerPath2.get_initial_pose().y)),
                    Swerve.followTrajectory(self.centerPath2),
                    commands2.cmd.parallel(
                        commands2.cmd.sequence(Swerve.followTrajectory(self.centerAlgae1), Swerve.followTrajectory(self.centerAlgae2)),
                        self.coralHandler.intakeL2_5())
                ).schedule()
            case self.leftAutoAlgae:
                initial_pose = self.leftPath1.get_initial_pose()
                commands2.cmd.sequence(
                    Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
                    commands2.cmd.parallel(
                        Swerve.followTrajectory(self.leftPath1),
                        self.coralHandler.goL4Command()),
                    self.scoreLeftCmd,
                    Swerve.resetPositionCmd(complex(self.leftPath2.get_initial_pose().x, self.leftPath2.get_initial_pose().y)),
                    Swerve.followTrajectory(self.leftPath2),
                    commands2.cmd.parallel(
                        commands2.cmd.sequence(Swerve.followTrajectory(self.leftAlgae1), Swerve.followTrajectory(self.leftAlgae2)),
                        self.coralHandler.intakeL3_5())
                ).schedule()
            case self.rightAutoAlgae:
                initial_pose = self.rightPath1.get_initial_pose()
                commands2.cmd.sequence(
                    Swerve.resetPoseCmd(complex(initial_pose.x, initial_pose.y), initial_pose.rotation().radians()),
                    commands2.cmd.parallel(
                        Swerve.followTrajectory(self.rightPath1),
                        self.coralHandler.goL4Command()),
                    self.scoreRightCmd,
                    Swerve.resetPositionCmd(complex(self.rightPath2.get_initial_pose().x, self.rightPath2.get_initial_pose().y)),
                    Swerve.followTrajectory(self.rightPath2),
                    commands2.cmd.parallel(
                        commands2.cmd.sequence(Swerve.followTrajectory(self.rightAlgae1), Swerve.followTrajectory(self.rightAlgae2)),
                        self.coralHandler.intakeL3_5())
                ).schedule()


if __name__ == "__main__":
    wpilib.run(Robot)