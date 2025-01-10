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
        from subsystems.photon_vision_camera import PhotonVisionCamera
        self.camera = PhotonVisionCamera("front_camera")
        self.leftCamera = PhotonVisionCamera("left_camera")

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

        aButton = JoystickButton(self.driverController, XboxController.Button.kA)

        from commands.setcamerapipeline import SetCameraPipeline
        setPipeline = SetCameraPipeline(self.camera, pipelineIndex=0)

        from commands.findobject import FindObject
        findTag = FindObject(self.camera, self.robotDrive, turnDegrees=-30)

        from commands.followobject import FollowObject, StopWhen
        followObject = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=5.0), speed=0.2)

        from commands.alignwithtag import AlignWithTag
        alignWithTag = AlignWithTag(self.camera, self.robotDrive, pushForwardSeconds=1, pushForwardSpeed=0.08, specificHeadingDegrees=0)

        aButton.whileTrue(setPipeline.andThen(findTag).andThen(followObject).andThen(alignWithTag))


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

        from commands.findobject import FindObject
        followTag = FindObject(self.leftCamera, self.robotDrive)

        #from commands.followobject import FollowObject, StopWhen
        #followTag = FollowObject(self.camera, self.robotDrive, stopWhen=StopWhen(maxSize=8.0), speed=0.2)

        command = setStartPose.andThen(followTag)
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
