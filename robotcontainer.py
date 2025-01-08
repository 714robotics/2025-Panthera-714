from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.aimtodirection import AimToDirection
from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem

from commands.reset_xy import ResetXY, ResetSwerveFront

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        from subsystems.limelight_camera import LimelightCamera
        self.camera = LimelightCamera("limelight-aiming")  # name of your camera goes in parentheses

        self.robotDrive = DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    True,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        from commands.setcamerapipeline import SetCameraPipeline
        from commands.followobject import FollowObject, StopWhen
        from commands.alignwithtag import AlignWithTag
        from commands.swervetopoint import SwerveToSide

        aButton = JoystickButton(self.driverController, XboxController.Button.kA)
        aButton.whileTrue(
            FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=4), speed=0.2)
            .andThen(
                AlignWithTag(self.camera, self.robotDrive, 0, speed=0.2, pushForwardSeconds=1.0)
            ).andThen(
                SwerveToSide(metersToTheLeft=-0.2, drivetrain=self.robotDrive, speed=1.0)
            ))

        JoystickButton(self.driverController, XboxController.Button.kLeftBumper).whileTrue(
            #AimToDirection(degrees=+60, drivetrain=self.robotDrive)
            SwerveToSide(0.8, drivetrain=self.robotDrive, speed=0.3)
        )

        JoystickButton(self.driverController, XboxController.Button.kRightBumper).whileTrue(
            #AimToDirection(degrees=-60, drivetrain=self.robotDrive)
            SwerveToSide(-0.8, drivetrain=self.robotDrive, speed=0.3)
        )

#        aButton.whileTrue(
#            FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=4), speed=0.2)
#        )

        xButton = JoystickButton(self.driverController, XboxController.Button.kX)
        xButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))
        xButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))  # use the swerve X brake when "X" is pressed

        yButton = JoystickButton(self.driverController, XboxController.Button.kY)
        yButton.onTrue(ResetSwerveFront(self.robotDrive))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.addOption("straight blue left", self.getAutonomousStraightBlueLeft)
        self.chosenAuto.addOption("curved blue left", self.getAutonomousCurvedBlueLeft)
        self.chosenAuto.setDefaultOption("approach tag", self.getApproachTagCommand)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getApproachTagCommand(self):
        setStartPose = ResetXY(x=6.775, y=7.262, headingDegrees=135, drivetrain=self.robotDrive)

        from commands.jerkytrajectory import JerkyTrajectory
        trajectory = JerkyTrajectory(
            drivetrain=self.robotDrive,
            endpoint=(2.594, 3.996, 0.0),
            waypoints=[
                (6.775, 7.262, 180.0),
                (6.503, 7.262, -178.831),
                (3.578, 7.106, -175.504),
                (2.174, 5.497, -101.094),
            ],
            speed=0.7,
            swerve=False,
        )

        from commands.followobject import FollowObject, StopWhen
        followTag = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=8.0), speed=0.2)

        from commands.arcadedrive import ArcadeDrive
        driveForwardALittle = ArcadeDrive(driveSpeed=0.15, rotationSpeed=0.0, drivetrain=self.robotDrive).withTimeout(0.3)

        command = setStartPose.andThen(trajectory) #.andThen(followTag).andThen(driveForwardALittle)
        return command


    def getAutonomousStraightBlueLeft(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)

        from commands.gotopoint import GoToPoint
        driveForward = GoToPoint(x=6.33, y=6.686, drivetrain=self.robotDrive)

        command = setStartPose.andThen(driveForward)
        return command

    def getAutonomousCurvedBlueLeft(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getAutonomousTrajectoryExample(self) -> commands2.Command:
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(0.5, 0.5), Translation2d(1, -0.5)],
            # End 1.5 meters straight ahead of where we started, facing forward
            Pose2d(1.5, 0, Rotation2d(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        driveController = HolonomicDriveController(
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPXController, 0, 0),
            thetaController,
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            driveController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
