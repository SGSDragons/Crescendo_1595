// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.utilities.Constants;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.LimelightTarget;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AimSimple extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private LimelightTarget speakerTarget;
  private double rotationValue;

  LimelightTarget.Error targetLock;
  double xErr;

  public AimSimple(
      LimelightTarget speakerTarget, DrivetrainSubsystem drivetrainSubsystem)
  {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.speakerTarget = speakerTarget;
    rotationValue = 0;
  }

  @Override
  public void initialize() {
   
      LimelightHelpers.setLEDMode_ForceOn("limelight");
  }

  @Override
  public void execute() {

      targetLock = speakerTarget.find(0);
      if (targetLock != null) {
        xErr = MathUtil.applyDeadband(targetLock.x, 2);
        if (Math.abs(xErr) > 5.0) {
           if (xErr > 0) {
              xErr = 5.0;
           }
           else {
             xErr = -5.0;
           }

        }
          rotationValue = xErr * 0.005;
      }


    drivetrainSubsystem.drive(
      new Translation2d(0.0, 0.0),
      rotationValue * Constants.SwerveConstants.maxAngularVelocity,
      true,
      true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(
      new Translation2d(0.0, 0.0),
      0,
      true,
      true);
  }

  @Override
  public boolean isFinished() {
      if (xErr == 0) {
        return true;
      }
      return false;
  }
}
