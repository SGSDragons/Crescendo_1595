// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.utilities.Constants;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.LimelightTarget;
import frc.lib.utilities.Utilities;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAim extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;

  private LimelightTarget speakerTarget;
  private LimelightTarget.Error targetLock;
  private double xError;
  private double rotationValue;

  public AutoAim(LimelightTarget speakerTarget, DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.speakerTarget = speakerTarget;
    rotationValue = 0.0;
  }

  @Override
  public void initialize() {
      LimelightHelpers.setLEDMode_ForceOn("limelight");
  }

  @Override
  public void execute() {
      targetLock = speakerTarget.find(0.0);

      //If the xError is within 2 of 0, sets it to 0. Then, clamps the value between 5 and -5. Finally, divides the value by 5 to get a range of -1 to 1.
      if (targetLock != null) {
        xError = MathUtil.applyDeadband(targetLock.x, 2);
        Utilities.rangeClamp(xError, 5, -5);
        xError *= 0.20;
        //Gain is < 1 so as to lessen the rotationValue and, thus, the speed of the rotation.
        rotationValue = xError * 0.005;
      }
      //If no tag is detected, if a rotation value was previously found, just mainting course at the smallest value. Otherwise, the command will end (no initial detection).
      else {
        if (rotationValue != 0.0) {
          Utilities.rangeClamp(rotationValue, 0.002, -0.002);
        }
      }

    drivetrainSubsystem.drive(
      new Translation2d(0.0, 0.0),
      rotationValue * Constants.SwerveConstants.maxAngularVelocity,
      true,
      true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    LimelightHelpers.setLEDMode_ForceOff("limelight");
  }

  @Override
  public boolean isFinished() {
    if (rotationValue == 0.0) {
      return true;
    }
    return false;
  }
}
