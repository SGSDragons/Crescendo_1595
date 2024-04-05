// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.utilities.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDrive extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier robotCentricSupplier;

  private double translationValue, strafeValue, rotationValue;

  public TeleopDrive(
      DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;
      }
      
  @Override
  public void execute() {
    translationValue = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);
    strafeValue = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);
    rotationValue = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);

    drivetrainSubsystem.drive(
      new Translation2d(translationValue, strafeValue).times(Constants.SwerveConstants.maxSpeed),
      rotationValue * Constants.SwerveConstants.maxAngularVelocity,
      !robotCentricSupplier.getAsBoolean(),
      true);
  }
}
