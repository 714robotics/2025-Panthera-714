#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import math
import typing
import commands2

from commands.aimtodirection import AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

from wpimath.geometry import Rotation2d
from wpilib import Timer

from subsystems.limelight_camera import LimelightCamera


class AlignSwerveWithTag(commands2.Command):
    FINISH_ALIGNMENT_EXTRA_SECONDS = 0.5  # at least 0.5 seconds to finish the alignment
    TOLERANCE_METERS = 0.025  # one inch tolerance for alignment

    def __init__(self, camera, drivetrain, headingDegrees, speed=0.2, pushForwardSeconds=0.0, reverse=False):
        """
        Will align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain:
        :param headingDegrees:
        :param speed: if the camera is on the back of the robot, please use negative speed
        :param pushForwardSeconds: if you want the robot to push forward at the end of alignment
        :param reverse: set it =True if the camera is on the back of the robot (not front)
        """
        super().__init__()
        assert hasattr(camera, "getX"), "camera must have `getX()` to give us the object coordinate (in degrees)"
        assert hasattr(camera, "getA"), "camera must have `getA()` to give us object size (in % of screen)"
        assert hasattr(camera, "getSecondsSinceLastHeartbeat"), "camera must have a `getSecondsSinceLastHeartbeat()`"
        assert hasattr(drivetrain, "drive"), "drivetrain must have a `drive()` function, because we need a swerve drive"
        assert pushForwardSeconds >= 0, f"pushForwardSeconds={pushForwardSeconds}, but must be >=0"

        self.drivetrain = drivetrain
        self.camera = camera

        self.addRequirements(drivetrain)
        self.addRequirements(camera)

        self.reverse = reverse
        self.speed = min((1.0, abs(speed)))  # ensure that the speed is between 0.0 and 1.0
        self.pushForwardSeconds = pushForwardSeconds

        # setting the target angle in a way that works for all cases
        self.targetDegrees = headingDegrees
        if headingDegrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(headingDegrees):
            self.targetDegrees = lambda: headingDegrees

        # state
        self.targetDirection = None
        self.alignedToTag = False
        self.pushForwardUntilTime = None
        self.haveNotSeenObjectSinceTime = None


    def initialize(self):
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())
        self.alignedToTag = False
        self.pushForwardUntilTime = None
        self.haveNotSeenObjectSinceTime = Timer.getFPGATimestamp()

    def isFinished(self) -> bool:
        now = Timer.getFPGATimestamp()
        if now > self.haveNotSeenObjectSinceTime + 2.0:
            print("AlignSwerveWithTag: finished because have not seen the object for >2 seconds")
            return True
        if self.alignedToTag and self.pushForwardSeconds == 0:
            print("AlignSwerveWithTag: finished because aligned to the tag and don't need to push forward")
            return True
        if self.alignedToTag and self.pushForwardUntilTime is not None and now > self.pushForwardUntilTime:
            print("AlignSwerveWithTag: finished because aligned to the tag and the forward push is finished")
            return True


    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)


    def execute(self):
        if self.camera.hasDetection():
            self.haveNotSeenObjectSinceTime = Timer.getFPGATimestamp()

        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed:
            turnSpeed = AimToDirectionConstants.kMinTurnSpeed  # but not too small

        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed

        # 4. if the robot heading is almost aligned, start swerving right or left (for centering on that tag precisely)
        swerveSpeed = 0
        if abs(degreesRemaining) < 4 * AimToDirectionConstants.kAngleToleranceDegrees:
            swerveSpeed = self.getSwerveLeftSpeed()

        # 5. if we just aligned to the tag and should be pushing forward, make that push
        pushForwardSpeed = 0.0
        if self.alignedToTag and self.pushForwardSeconds > 0:
            pushForwardSpeed = self.getPushForwardSpeed()

        # 6. all the speeds are computed, drive with those speeds
        self.drivetrain.drive(pushForwardSpeed, swerveSpeed, turnSpeed, fieldRelative=False, rateLimit=False)


    def getPushForwardSpeed(self):
        pushForwardSpeed = self.speed
        if pushForwardSpeed < GoToPointConstants.kMinTranslateSpeed:
            pushForwardSpeed = GoToPointConstants.kMinTranslateSpeed
        if self.reverse:
            pushForwardSpeed = -pushForwardSpeed
        if self.pushForwardUntilTime is None:
            self.pushForwardUntilTime = Timer.getFPGATimestamp() + self.pushForwardSeconds
        return pushForwardSpeed


    def getSwerveLeftSpeed(self):
        swerveSpeed = 0.0
        secondsSinceHeartbeat = self.camera.getSecondsSinceLastHeartbeat()
        objectSizePercent = self.camera.getA()
        objectXDegrees = self.camera.getX()
        if secondsSinceHeartbeat > 0.5:
            print(f"WARNING: in AlignSwerveWithTag, camera not usable (dead or too few frames per second), {secondsSinceHeartbeat} seconds since last hearbeat")
        elif objectXDegrees == 0 or objectSizePercent <= 0:
            print(f"WARNING: in AlignSwerveWithTag, invalid camera (objectX, objectSize) = ({objectXDegrees}, {objectSizePercent})")
        else:
            swerveSpeed, objectXMeters = self.calculateSwerveLeftSpeed(objectSizePercent, objectXDegrees)

            if abs(swerveSpeed) <= GoToPointConstants.kMinTranslateSpeed:
                print(f"AlignSwerveWithTag: almost done, since swerve speed {swerveSpeed} is already small")

                if abs(objectXMeters) < AlignSwerveWithTag.TOLERANCE_METERS:
                    print(f"AlignSwerveWithTag: objectXDegrees={objectXDegrees} is small too, we are done")
                    self.alignedToTag = True
        return swerveSpeed


    def calculateSwerveLeftSpeed(self, objectSizePercent, objectXDegrees):
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 4-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (4 * 0.01)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        distanceMeters = math.sqrt(0.2 * 0.2 / (4 * 0.01 * objectSizePercent))

        # trigonometry: how many meters on the left is our object? (if negative, then it's on the right)
        objectXMeters = -distanceMeters * Rotation2d.fromDegrees(objectXDegrees).tan()
        # if the camera is on the back of the robot, then right and left are swapped
        if self.reverse:
            objectXMeters = -objectXMeters

        # how fast should we swerve to the right or left? use proportional control!
        swerveSpeed = self.speed
        proportionalSpeed = GoToPointConstants.kPTranslate * abs(objectXMeters)
        if proportionalSpeed < swerveSpeed:
            swerveSpeed = proportionalSpeed
        if swerveSpeed < GoToPointConstants.kMinTranslateSpeed and self.pushForwardSeconds == 0:
            swerveSpeed = GoToPointConstants.kMinTranslateSpeed

        # if the object is on the right, swerve to the right (negative swerve speed)
        if objectXMeters < 0:
            swerveSpeed = -swerveSpeed

        return swerveSpeed, objectXMeters
