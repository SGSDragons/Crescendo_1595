// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.utilities.Constants;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.LimelightTarget;
import frc.lib.utilities.Utilities;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopAim extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier robotCentricSupplier;
  private DoubleConsumer controllerRumble;

  private LimelightTarget speakerTarget;
  private LimelightTarget.Error targetLock;
  private double xError;

  private double translationValue, strafeValue;
  private double rotationValue = 0.0;

  public TeleopAim(
      DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier, DoubleConsumer controllerRumble) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
        this.controllerRumble = controllerRumble;
      }
      
  @Override
  public void initialize() {
    LimelightHelpers.setLEDMode_ForceOn("limelight");
    this.speakerTarget = new LimelightTarget(RobotContainer.isBlue ? 7: 4, 0.0, 0.0, 0.0);
  }

  @Override
  public void execute() {
    translationValue = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);
    strafeValue = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);
    
    targetLock = speakerTarget.find(0.0);
    //If a target is found, rotationValue is purely based off error.
    if (targetLock != null) {
      xError = MathUtil.applyDeadband(targetLock.x, 2);
      Utilities.rangeClamp(xError, 5, -5);
      xError *= 0.20;
      rotationValue = xError * 0.005;

      controllerRumble.accept(1.0 - xError);
    }

    else {
      controllerRumble.accept(0.0);
      //If a previous non-zero rotationValue existed (previosly identified tag), just continue moving in the same direction slowly in an effort to re-detect tag.
      if (rotationValue != 0.0) {
        Utilities.rangeClamp(rotationValue, 0.002, -0.002);
      }

      //Otherwise, normal joystick control is utilized (never detected tag).
      else {
        rotationValue = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);
      }
    }

    drivetrainSubsystem.drive(
      new Translation2d(translationValue, strafeValue).times(Constants.SwerveConstants.maxSpeed),
      rotationValue * Constants.SwerveConstants.maxAngularVelocity,
      !robotCentricSupplier.getAsBoolean(),
      true);
  }

  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    controllerRumble.accept(0.0);
  }
}
