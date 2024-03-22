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

public class TeleopDrive extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier robotCentricSupplier;
  private DoubleConsumer rumble;
  private BooleanSupplier autoAim;
  private LimelightTarget speakerTarget;

  public TeleopDrive(
      DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translaSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier,
      BooleanSupplier autoAim,
      DoubleConsumer rumble
    )
  {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    this.translationSupplier = translaSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.robotCentricSupplier = robotCentricSupplier;
    this.autoAim = autoAim;
    this.rumble = rumble;

    boolean isBlue = DriverStation.getAlliance().filter(a -> a == DriverStation.Alliance.Blue).isPresent();
    this.speakerTarget = new LimelightTarget(isBlue ? 7 : 3, 0, 6.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);
    double strafeValue = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);
    double rotationValue = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.OperatorConstants.stickDeadband);

    if (autoAim.getAsBoolean()) {
      LimelightHelpers.setLEDMode_ForceOn("limelight");
      LimelightTarget.Error targetLock = speakerTarget.find(0); // Heading is unused
      if (targetLock != null) {
        double xErr = MathUtil.applyDeadband(targetLock.x, 3);
        if (Math.abs(xErr) > 10.0) {
          xErr *= 10.0/xErr;
        }
        if (xErr == 0.0) {
          rumble.accept(1.0);
        } else {
          rotationValue += xErr * Aim.headingGain.getDouble(0.0);
        }
      }
    } else {
      rumble.accept(0.0);
      LimelightHelpers.setLEDMode_ForceOff("limelight");
    }

    drivetrainSubsystem.drive(
      new Translation2d(translationValue, strafeValue).times(Constants.SwerveConstants.maxSpeed),
      rotationValue * Constants.SwerveConstants.maxAngularVelocity,
      !robotCentricSupplier.getAsBoolean(),
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
