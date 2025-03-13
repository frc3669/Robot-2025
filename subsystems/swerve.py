import cmath, commands2, choreo
from wpilib import Timer, SmartDashboard, Joystick, DriverStation, DigitalInput
from phoenix6 import hardware
from subsystems.swerveModule import SwerveModule, constants, mf
from typing import final

@final
class Swerve(commands2.Subsystem):
    gyro = hardware.Pigeon2(1, "CTREdevices")
    poleSensor = DigitalInput(1)
    auto_timer = Timer()
    modules: list[SwerveModule] = []
    slew_velocity = complex()
    slew_angular_velocity = 0
    position = 0 + 0j
    startingAngle = 0
    heading = 0
    trajectory = None
    possible_reef_angles = [x*cmath.pi/180 for x in range(0, 360, 60)]
    possible_feeder_station_angles = [2.1995556168958954, -2.1995556168958954]
    print(possible_reef_angles)
    
    @staticmethod
    def simulationPeriodic():
        pass

    @staticmethod
    def driveTeleop(controller: Joystick, reefAlignEnabled = False, feederStationAlignEnabled = False):
        velocity = 0
        angular_velocity = 0
        if controller.getName() == "Controller (Xbox One For Windows)":
            velocity = complex(-controller.getRawAxis(1), -controller.getRawAxis(0))
            angular_velocity = -controller.getRawAxis(4)
            if controller.getRawButton(4):
                Swerve.gyro.set_yaw(0)
        elif controller.getName() == "Radiomaster Boxer Joystick":
            velocity = complex(controller.getRawAxis(0), controller.getRawAxis(1))
            angular_velocity = controller.getRawAxis(2)
            if controller.getRawButton(4):
                Swerve.gyro.set_yaw(0)
        # apply smooth deadband
        dB = 0.03
        if abs(velocity) > dB:
            velocity *= (1 - dB/abs(velocity))/(1 - dB)
        else:
            velocity = complex(0, 0)
        if abs(angular_velocity) > dB:
            angular_velocity *= (1 - dB/abs(angular_velocity))/(1 - dB)
        else:
            angular_velocity = 0
        # angle the robot to the closest face of the reef
        Swerve.heading = Swerve.gyro.get_yaw().value_as_double*cmath.tau/360 + Swerve.startingAngle
        if reefAlignEnabled:
            angular_velocity += Swerve.getReefAlignmentError() * constants.swerve_autoalign_P
        elif feederStationAlignEnabled:
            angular_velocity += Swerve.getFeederStationAlignmentError() * constants.swerve_autoalign_P
        # scale the velocities to meters per second
        velocity *= constants.max_m_per_sec
        angular_velocity *= constants.max_m_per_sec
        # find the robot oriented velocity
        robot_velocity = velocity * cmath.rect(1, -Swerve.heading)
        # find the fastest module speed
        highest = constants.max_m_per_sec
        for module in Swerve.modules:
            module_speed = abs(module.find_module_vector(robot_velocity, angular_velocity))
            if module_speed > highest:
                highest = module_speed
        # scale the velocities
        velocity *= constants.max_m_per_sec/highest
        angular_velocity *= constants.max_m_per_sec/highest
        robot_velocity *= constants.max_m_per_sec/highest
        # find the error between the command and the current velocities
        velocity_error = velocity - Swerve.slew_velocity
        angular_velocity_error = angular_velocity - Swerve.slew_angular_velocity
        # find the robot oriented velocity error
        robot_velocity_error = velocity_error * cmath.rect(1, -Swerve.heading)
        robot_slew_velocity = Swerve.slew_velocity * cmath.rect(1, -Swerve.heading)
        # find the max acceleration overshoot
        highest = 1
        for module in Swerve.modules:
            module_overshoot = module.get_accel_overshoot(robot_slew_velocity, Swerve.slew_angular_velocity, robot_velocity_error, angular_velocity_error)
            if module_overshoot > highest:
                highest = module_overshoot
        # find velocity increments
        velocity_increment = velocity_error/highest
        angular_velocity_increment = angular_velocity_error/highest
        # increment velocity
        if abs(velocity_error) > constants.max_m_per_sec_per_cycle:
            Swerve.slew_velocity += velocity_increment
        else:
            Swerve.slew_velocity = velocity
        if abs(angular_velocity_error) > constants.max_m_per_sec_per_cycle:
            Swerve.slew_angular_velocity += angular_velocity_increment
        else:
            Swerve.slew_angular_velocity = angular_velocity
        # update the robot oriented slew velocity
        robot_slew_velocity = Swerve.slew_velocity * cmath.rect(1, -Swerve.heading)
        # find acceleration feedforward
        robot_accel = robot_velocity_error*2
        angular_accel = angular_velocity_error*2
        # drive the modules
        for module in Swerve.modules:
            module.set_velocity(robot_slew_velocity, Swerve.slew_angular_velocity, robot_accel, angular_accel)
        Swerve.calculateOdometry()

    @staticmethod
    def testAccel(controller: Joystick, test_value):
        acceleration = 0
        dB = 0.8
        if controller.getRawAxis(0) > dB:
            acceleration = test_value
        if controller.getRawAxis(0) < -dB:
            acceleration = -test_value
        SmartDashboard.putNumber('torque current setpoint', acceleration)
        for module in Swerve.modules:
            module.accelTest(acceleration)
        SmartDashboard.putNumber('timestamp', Timer.getFPGATimestamp())
    
    @staticmethod
    def setTrajectory(trajectory: choreo.SwerveTrajectory):
        Swerve.trajectory = trajectory
        Swerve.auto_timer.restart()

    @staticmethod  
    def moveToNextSample():
        Swerve.heading = Swerve.gyro.get_yaw().value_as_double*cmath.tau/360 + Swerve.startingAngle
        # find the latest sample index
        if not Swerve.auto_timer.hasElapsed(Swerve.trajectory.get_total_time()):
            Swerve.calculateOdometry()
            current_sample = Swerve.trajectory.sample_at(Swerve.auto_timer.get())
            # calculate the proportional response
            position_error = complex(current_sample.x, current_sample.y) - Swerve.position
            heading_error = current_sample.heading - Swerve.heading
            heading_error = mf.get_wrapped(heading_error)
            velocity = complex(current_sample.vx, current_sample.vy) + constants.swerve_position_P * position_error
            angular_velocity = current_sample.omega + constants.swerve_heading_P * heading_error
            velocity *= cmath.rect(1, -Swerve.heading)
            for module in Swerve.modules:
                module.set_velocity(velocity, angular_velocity, complex(current_sample.ax, current_sample.ay), current_sample.alpha)
        else:
            for module in Swerve.modules:
                module.set_velocity()

    @staticmethod
    def followTrajectory(trajectory: choreo.SwerveTrajectory) -> commands2.Command:
        return commands2.FunctionalCommand (
            lambda: Swerve.setTrajectory(trajectory), 
            lambda: Swerve.moveToNextSample(),
            lambda x : Swerve.brake(),
            lambda: Swerve.auto_timer.hasElapsed(trajectory.get_total_time()),
            Swerve)

    @staticmethod
    def simpleDrive(velocity: complex, angular_velocity: float = 0):
        highest = constants.max_m_per_sec
        for module in Swerve.modules:
            module_speed = abs(module.find_module_vector(velocity, angular_velocity))
            if module_speed > highest:
                highest = module_speed
        velocity *= constants.max_m_per_sec/highest
        angular_velocity *= constants.max_m_per_sec/highest
        for module in Swerve.modules:
            module.set_velocity(velocity, angular_velocity)

    @staticmethod
    def brake():
        for module in Swerve.modules:
            module.brake()

    def driveRightToPole() -> commands2.Command:
        return commands2.FunctionalCommand (
            lambda: Swerve.simpleDrive(0-0.3j),
            lambda: Swerve.simpleDrive(0-0.3j),
            lambda x : Swerve.simpleDrive(0),
            lambda: not Swerve.poleSensor.get(),
            Swerve)

    def driveLeftToPole() -> commands2.Command:
        return commands2.FunctionalCommand (
            lambda: Swerve.simpleDrive(0+0.3j),
            lambda: Swerve.simpleDrive(0+0.3j),
            lambda x : Swerve.simpleDrive(0),
            lambda: not Swerve.poleSensor.get(),
            Swerve)
    
    def getReefAlignmentError() -> float:
        for angle in Swerve.possible_reef_angles:
            error = mf.get_wrapped(angle - Swerve.heading)
            if abs(error) <= cmath.pi/6:
                return error
        return 0
    
    def getFeederStationAlignmentError() -> float:
        error1 = mf.get_wrapped(Swerve.possible_feeder_station_angles[0] - Swerve.heading)
        error2 = mf.get_wrapped(Swerve.possible_feeder_station_angles[1] - Swerve.heading)
        if abs(error1) < abs(error2):
            return error1
        return error2
    
    def resetPoseCmd(new_position: complex, new_angle: float) -> commands2.Command:
            return commands2.InstantCommand (
                lambda: Swerve.resetPose(new_position, new_angle),
                Swerve)
    
    def resetPositionCmd(new_position: complex) -> commands2.Command:
        return commands2.InstantCommand (
            lambda: Swerve.resetPosition(new_position),
            Swerve
        )
    
    @staticmethod
    def getTrajectoryRemainingTime():
        return Swerve.trajectory.getEndTime() - Swerve.auto_timer.get()

    @staticmethod
    def calculateOdometry():
        position_change = complex(0, 0)
        for module in Swerve.modules:
            position_change += module.getPositionChange()
        Swerve.position += position_change * cmath.rect(0.25, Swerve.heading)

    @staticmethod
    def add_module(module: SwerveModule):
        Swerve.modules.append(module)

    @staticmethod
    def resetPosition(new_position: complex = complex()):
        for module in Swerve.modules:
            module.reset_encoders()
        Swerve.position = new_position
    
    def resetPose(new_position: complex, new_angle: float):
        for module in Swerve.modules:
            module.reset_encoders()
        Swerve.position = new_position
        Swerve.startingAngle = new_angle
    
    @staticmethod
    def getRotationRate():
        angularRate = 0
        for module in Swerve.modules:
            angularRate += module.getRotationContribution()
        return angularRate / 4
    
    def doNothing(x = False):
        pass