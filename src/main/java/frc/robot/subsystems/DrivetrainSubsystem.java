// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VoltageOut;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.utilities.Constants.AutoConstants;
import frc.lib.utilities.Constants.SwerveConstants;
import frc.lib.utilities.swerve.COTSTalonFXSwerveConstants.SDS.MK4;
import frc.lib.utilities.swerve.Conversions;
import frc.lib.utilities.swerve.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
  public SwerveDriveOdometry swerveDriveOdometry;
  public SwerveModule[] swerveModules;
  public AHRS navx;
  private double highestMeasuredVelocity = 0;

  //System Identification

  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
  private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Units.Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));

  private SysIdRoutine sysIdRoutine;
  private VoltageOut swerveModuleVoltageOutput = new VoltageOut(0.0);

  public DrivetrainSubsystem() {
    //Initializes Gyroscope (Measures Yaw/Rotation Angle), swerve modules, and swerve odometry.
    navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    navx.reset();

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.Mod0.constants),
      new SwerveModule(1, SwerveConstants.Mod1.constants),
      new SwerveModule(2, SwerveConstants.Mod2.constants),
      new SwerveModule(3, SwerveConstants.Mod3.constants)
    };
    
    swerveDriveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
    createSytemIdentificationRoutine();

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetHeading,
      this::getChassisSpeeds,
      this::drive,
      AutoConstants.pathPlannerConfig,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this);
  }

  public void drive(Translation2d translation, double anglularVelocity, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
      SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                anglularVelocity,
                getHeading()
              )
              : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                anglularVelocity)
              );
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

      for (SwerveModule module : swerveModules) {
        module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
      }
  }

  public void drive (ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates =
      SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
    }
  }

  // Used by SwerveControllerCommand in Auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }

    return positions;
  }

  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public void resetHeading(Pose2d pose) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(-navx.getAngle()); //Negated because NavX CW Positive while ChassisSpeeds requires CCW Positive
  }

  public void resetModulesToAbsolute() {
    for(SwerveModule module : swerveModules) {
      module.resetToAbsolute();
    }
  }

  private void telemetry() {
    for(SwerveModule module : swerveModules){
            SmartDashboard.putNumber("Module " + module.moduleNumber + " CANcoder", module.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle", module.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
            if (module.getState().speedMetersPerSecond > highestMeasuredVelocity) {
              highestMeasuredVelocity = module.getState().speedMetersPerSecond;
            }
          }     
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("MaximumSpeed", highestMeasuredVelocity);
  }

  // Makes System Identification Routine for Mathematical Analysis. Runs two tests that apply specific voltages to motors and log their positions and velocities.
  private void createSytemIdentificationRoutine() {
    sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(0.75), Units.Volts.of(0.75), Units.Seconds.of(5)),
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        for(SwerveModule module : swerveModules) {
          module.driveMotor.setControl(swerveModuleVoltageOutput.withOutput(volts.in(Units.Volts)));
        }},
        
      log -> {
        for (SwerveModule module : swerveModules) {
          log.motor(Integer.toString(module.moduleNumber))
            .voltage(appliedVoltage.mut_replace(module.driveMotor.getMotorVoltage().getValueAsDouble() * RobotController.getBatteryVoltage(), Units.Volts))
            .linearPosition(distance.mut_replace(Conversions.rotationsToMeters(module.driveMotor.getPosition().getValueAsDouble() * MK4.driveRatios.L1, edu.wpi.first.math.util.Units.inchesToMeters(4.0) * Math.PI),Units.Meters))
            .linearVelocity(velocity.mut_replace(module.driveMotor.getVelocity().getValueAsDouble(), Units.MetersPerSecond));
        }
      },
      
      this));
  }

  // Drive System Identification Commands for Testing and Analysis
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    swerveDriveOdometry.update(getGyroYaw(), getModulePositions());
    //SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    //telemetry();
  }
}
