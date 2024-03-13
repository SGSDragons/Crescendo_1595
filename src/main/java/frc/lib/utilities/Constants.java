// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.utilities;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.utilities.swerve.COTSTalonFXSwerveConstants;
import frc.lib.utilities.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OperatorConstants {

    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final double stickDeadband = 0.1;

    // Button names
    public static final int backLeftSingleSwitch = 1;
    public static final int topLeftToggleBack = 2;
    public static final int topLeftToggleForward = 3;
    public static final int frontFarLeftToggleUp = 4;
    public static final int frontFarLeftToggleDown = 5;
    public static final int frontNearLeftToggleUp = 6;
    public static final int frontNearLeftToggleDown = 7;
    public static final int frontRightToggleDown = 8; 
    public static final int frontRightToggleUp = 9;
    public static final int topRightToggleForward = 10;
    public static final int topRightToggleBack = 11;
    public static final int backRightSingleSwitch = 12;
    public static final int topLeftButton = 13; 
    public static final int resetButton = 14;
    public static final int cancelButton = 15;
    public static final int rollerButton = 16;
    public static final int rollerLeft = 17;
    public static final int rollerRight = 18;
    public static final int frontLeftBottomSwitchLeft = 19;
    public static final int frontLeftBottomSwitchRight = 20;
    public static final int frontLeftMiddleSwitchDown = 21;
    public static final int frontLeftMiddleSwitchUp = 22;
    public static final int frontRightBottomSwitchLeft = 23;
    public static final int frontRightBottomSwitchRight = 24;
    public static final int frontRightMiddleSwitchDown = 25;  
    public static final int frontRightMiddleSwitchUp = 26;
}

  //Hardware IDs for parts on robot (excluding drivetrain).
  public static final class HardwareID {
    public static final int leftClimberForwardChannel = 0;
    public static final int leftClimberReverseChannel = 1;
    public static final int rightClimberForwardChannel = 2;
    public static final int rightClimberReverseChannel = 3;
    public static final int noteAimerForwardChannel = 4;
    public static final int noteAimerReverseChannel = 5;

    public static final int indexerMotorCANId = 18;
    public static final int indexerMotor2CANId = 15;
    public static final int bottomSpinnerMotorCANId = 19;
    public static final int middleSpinnerMotorCANId = 17;
    public static final int topSpinnerMotorCANId = 16;
  }

  public static final class SystemToggles {
    public static final boolean systemIdentification = false;
  }
  
  public static final class SwerveConstants {
         public static final COTSTalonFXSwerveConstants chosenModule =
         COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L1);
  
         /* Drivetrain Constants */
         public static final double trackWidth = Units.inchesToMeters(21.5);
         public static final double wheelBase = Units.inchesToMeters(18.5);
         public static final double wheelCircumference = chosenModule.wheelCircumference;
  
         /* Swerve Kinematics 
          * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
          public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
             new Translation2d(wheelBase / 2.0, trackWidth / 2.0), 
             new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

          public static final double driveBaseRadius = new Translation2d(wheelBase / 2.0, trackWidth / 2.0).getNorm();
  
         /* Module Gear Ratios */
         public static final double driveGearRatio = chosenModule.driveGearRatio;
         public static final double angleGearRatio = chosenModule.angleGearRatio;
  
         /* Motor Inverts */
         public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
         public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;
  
         /* Angle Encoder Invert */
         public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;
  
         /* Swerve Current Limiting */
         public static final int angleCurrentLimit = 25;
         public static final int angleCurrentThreshold = 40;
         public static final double angleCurrentThresholdTime = 0.1;
         public static final boolean angleEnableCurrentLimit = true;
  
         public static final int driveCurrentLimit = 35;
         public static final int driveCurrentThreshold = 60;
         public static final double driveCurrentThresholdTime = 0.1;
         public static final boolean driveEnableCurrentLimit = true;
  
         /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
          * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
         public static final double openLoopRamp = 0.25;
         public static final double closedLoopRamp = 0.0;
  
         /* Angle Motor PID Values */
         public static final double angleKP = chosenModule.angleKP;
         public static final double angleKI = chosenModule.angleKI;
         public static final double angleKD = chosenModule.angleKD;
  
         /* Drive Motor PID Values */
         public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
         public static final double driveKI = 0.0;
         public static final double driveKD = 0.0;
         public static final double driveKF = 0.0;
  
         /* Drive Motor Characterization Values From SYSID */
         public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
         public static final double driveKV = 1.51;
         public static final double driveKA = 0.27;
  
         /* Swerve Profiling Values */
         /** Meters per Second */
         public static final double maxSpeed = 3.0; //TODO: This must be tuned to specific robot
         /** Radians per Second */
         public static final double maxAngularVelocity = 8.0; //TODO: This must be tuned to specific robot
  
         /* Neutral Modes */
         public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
         public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
  
         /* Module Specific Constants */
         /* Front Left Module - Module 0 */
         public static final class Mod0 {
             public static final int driveMotorID = 3;
             public static final int angleMotorID = 4;
             public static final int canCoderID = 5;
             public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-101.601563);
             public static final SwerveModuleConstants constants = 
                 new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
         }
  
         /* Front Right Module - Module 1 */
         public static final class Mod1 {
             public static final int driveMotorID = 6;
             public static final int angleMotorID = 7;
             public static final int canCoderID = 8;
             public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-58.095703);
             public static final SwerveModuleConstants constants = 
                 new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
         }
         
         /* Back Left Module - Module 2 */
         public static final class Mod2 {
             public static final int driveMotorID = 9;
             public static final int angleMotorID = 10;
             public static final int canCoderID = 11;
             public static final Rotation2d angleOffset = Rotation2d.fromDegrees(48.603516);
             public static final SwerveModuleConstants constants = 
                 new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
         }
  
         /* Back Right Module - Module 3 */
         public static final class Mod3 {
             public static final int driveMotorID = 12;
             public static final int angleMotorID = 13;
             public static final int canCoderID = 14;
             public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-153.632813);
             public static final SwerveModuleConstants constants = 
                 new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
         }
     }
  
     public static final class AutoConstants { //TO BE TUNED
         public static final double kMaxSpeedMetersPerSecond = 3;
         public static final double kMaxAccelerationMetersPerSecondSquared = 3;
         public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
         public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
     
         public static final double kPXController = 3;
         public static final double kPYController = 3;
         public static final double kPThetaController = 4;
     
         /* Constraint for the motion profilied robot angle controller */
         public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
             new TrapezoidProfile.Constraints(
                 kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    

          public static final HolonomicPathFollowerConfig pathPlannerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(AutoConstants.kPXController, 0, 0),
            new PIDConstants(AutoConstants.kPThetaController, 0, 0),
            SwerveConstants.maxSpeed,
            SwerveConstants.driveBaseRadius,
            new ReplanningConfig(false, false));
     }
}
