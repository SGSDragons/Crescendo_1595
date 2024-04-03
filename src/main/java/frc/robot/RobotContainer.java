// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;

import frc.lib.utilities.LimelightTarget;
import frc.robot.commands.*;
import frc.robot.commands.Index.IndexDirection;
import frc.robot.commands.Launch.LaunchDirection;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.lib.utilities.Constants.Keys;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.utilities.Constants.OperatorConstants;
import frc.lib.utilities.Constants.SystemToggles;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
  private static boolean compressorOnly = false;

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  public final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  private final Joystick driver = new Joystick(OperatorConstants.driverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.operatorControllerPort);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton slowAim = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  private final JoystickButton climberUp = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
  private final JoystickButton climberDown = new JoystickButton(operator, XboxController.Button.kRightStick.value);

  private final JoystickButton launchAmpAutomatic = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton launchAmpManual = new JoystickButton(operator, XboxController.Button.kBack.value);

  private final JoystickButton launchLowAutomatic = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton launchLowManual = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton launchHighAutomatic = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton launchHighManual = new JoystickButton(operator, XboxController.Button.kX.value);

  private final JoystickButton indexerIntake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton indexerOuttake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

  private final JoystickButton autoAim = new JoystickButton(driver, XboxController.Button.kA.value);
  
  private final JoystickButton compressor = new JoystickButton(driver, XboxController.Button.kX.value);

  //private final POVButton correctNotePosition = new POVButton(operator, 180);

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeRobotPreferences();
    registerNamedCommands();
    
    drivetrainSubsystem.setDefaultCommand(
      new TeleopDrive(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> -driver.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean(),
          () -> autoAim.getAsBoolean(),
          (intensity) -> {
            driver.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
            operator.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
          }
        )
    );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {

    //The keybinds and commands for system identification only load if the mode is enabled in constants (for programming purposes).
    if (Preferences.getBoolean(Keys.characterizationKey, false)) {
      JoystickButton driveQuasiForward = new JoystickButton(driver, XboxController.Button.kBack.value);
      JoystickButton driveQuasiBackward = new JoystickButton(driver, XboxController.Button.kStart.value);
      JoystickButton driveDynamicForward = new JoystickButton(driver, XboxController.Button.kX.value);
      JoystickButton driveDynamicBackward = new JoystickButton(driver, XboxController.Button.kB.value);
      
      
      driveQuasiForward.whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      driveQuasiBackward.whileTrue(drivetrainSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      driveDynamicForward.whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      driveDynamicBackward.whileTrue(drivetrainSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      
      //No Other Keybinds will Load in System Identification Mode, for no other keybinds will be assigned to actions.
      return;
    }

    //Keybinds for... actually driving the robot in TeleOP.

    zeroGyro.onTrue(new InstantCommand(() -> drivetrainSubsystem.zeroHeading()));

    slowAim.whileTrue(new TeleopDrive(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> -driver.getRawAxis(rotationAxis) * 0.25,
          () -> robotCentric.getAsBoolean(),
          () -> autoAim.getAsBoolean(),
          (intensity) -> {
            driver.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
            operator.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
          }
        ));

    //Launches Notes (Automatic launches after spinup, Manual only launches after spinup and 'up' on dpad)

    launchLowAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, true, operator));
    launchLowManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, false, operator));
    
    launchHighAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, true, operator));
    launchHighManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, false, operator));
    
    launchAmpAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, true, operator));
    launchAmpManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, false, operator));
    

    indexerIntake.whileTrue(new Index(indexerSubsystem, launcherSubsystem, IndexDirection.INTAKE));
    indexerIntake.onFalse(new CorrectNotePosition(indexerSubsystem));

    indexerOuttake.whileTrue(new Index(indexerSubsystem, launcherSubsystem, IndexDirection.OUTTAKE));

    climberUp.onTrue(new InstantCommand(() -> {
      pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.leftClimber);
      pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.rightClimber);
    }));
    climberDown.onTrue(new InstantCommand(() -> {
      pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.leftClimber);
      pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.rightClimber);
    }));

    compressor.onTrue(new InstantCommand(() -> pneumaticsSubsystem.toggleCompressor()));
  }
  
  private void registerNamedCommands() {
    NamedCommands.registerCommand("LaunchNoteLow", new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, true, operator).withTimeout(1.5));
    NamedCommands.registerCommand("LaunchNoteHigh", new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, true, operator).withTimeout(1.5));
    NamedCommands.registerCommand("LaunchNoteAmp", new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, true, operator).withTimeout(1.5));
    
    NamedCommands.registerCommand("Intake",
      new Index(indexerSubsystem, launcherSubsystem, IndexDirection.INTAKE).withTimeout(1.25));

    NamedCommands.registerCommand("IntakeLong",
      new Index(indexerSubsystem, launcherSubsystem, IndexDirection.INTAKE).withTimeout(7.5));

    NamedCommands.registerCommand("CorrectNotePosition", new CorrectNotePosition(indexerSubsystem).withTimeout(0.5));

    //Auto Aim Commands
    boolean isBlue = isBlue();
    List<LimelightTarget> targets = isBlue ? blueTargets() : redTargets();
    for (int i=0; i < targets.size(); ++i) {
      // Play with tolerance until we are happy it gets close enough fast enough.
      NamedCommands.registerCommand("aim-"+i, new AimSimple(targets.get(i), drivetrainSubsystem).withTimeout(0.75));
    }
  }

  private static boolean isBlue() {
    return DriverStation.getAlliance().filter(a -> a == DriverStation.Alliance.Blue).isPresent();
  }

  public Command getAutonomousCommand() {
    drivetrainSubsystem.zeroHeading();
    return autoChooser.getSelected();
  }

  public List<LimelightTarget> blueTargets() {
    NetworkTable sgs = NetworkTableInstance.getDefault().getTable("sgs");
    List<LimelightTarget> targets = new ArrayList<>();

    targets.add(new LimelightTarget(
        7,
        -5.385597,
        7.125707,
        -12.60));
        // sgs.getEntry("aim_0_tx").getDouble(0.0),
        // sgs.getEntry("aim_0_ty").getDouble(0.0),
        // sgs.getEntry("aim_0_heading").getDouble(-150.0)));
    targets.add(new LimelightTarget(
        7,
        -3.069734,
        5.153919,
        165.569999));
        // sgs.getEntry("aim_1_tx").getDouble(0.0),
        // sgs.getEntry("aim_1_ty").getDouble(0.0),
        // sgs.getEntry("aim_1_heading").getDouble(-160.0)));
    targets.add(new LimelightTarget(
        7,
        5.330149,
        7.127398,
        150.020));
        // sgs.getEntry("aim_2_tx").getDouble(0.0),
        // sgs.getEntry("aim_2_ty").getDouble(0.0),
        // sgs.getEntry("aim_2_heading").getDouble(-180.0)));

    return targets;
  }

  public List<LimelightTarget> redTargets() {
    NetworkTable sgs = NetworkTableInstance.getDefault().getTable("sgs");
    List<LimelightTarget> targets = new ArrayList<>();

    targets.add(new LimelightTarget(
            4,
            5.385597,
            7.125707,
            12.60));
            // sgs.getEntry("aim_0_tx").getDouble(0.0),
            // sgs.getEntry("aim_0_ty").getDouble(6.0),
            // sgs.getEntry("aim_0_heading").getDouble(150.0)));
    targets.add(new LimelightTarget(
            4,
            3.069734,
            5.153919,
            -165.569999));
            // sgs.getEntry("aim_1_tx").getDouble(0.0),
            // sgs.getEntry("aim_1_ty").getDouble(0.0),
            // sgs.getEntry("aim_1_heading").getDouble(160.0)));
    targets.add(new LimelightTarget(
            4,
            -5.330149,
            7.127398,
            -150.020));
            // sgs.getEntry("aim_2_tx").getDouble(0.0),
            // sgs.getEntry("aim_2_ty").getDouble(0.0),
            // sgs.getEntry("aim_2_heading").getDouble(180.0)));

    return targets;
  }

  //I think this is being a HUGE waste of resources while doing ABSOLUTELY NOTHING and frequently causing COMMAND SCHEDULER LOOP OVERRUN
  
  public void updateLLTargetTelemetry() {
    NetworkTableEntry focusT = NetworkTableInstance.getDefault().getTable("sgs").getEntry("ll_target");
    int focus = (int)focusT.getInteger(0);
    focusT.setInteger(focus);

    List<LimelightTarget> targets = isBlue() ? blueTargets() : redTargets();
    if (focus >= 0 && focus < targets.size()) {
      targets.get(focus).find(drivetrainSubsystem.getHeading().getDegrees());
    }
  } 
   

  public void initializeRobotPreferences() {
    // Driving
    Preferences.initDouble(Keys.angle_kPKey, 100.0);
    Preferences.initDouble(Keys.drive_kPKey, 0.12);
    Preferences.initDouble(Keys.drive_kSKey, 1.654475);
    Preferences.initDouble(Keys.drive_kVKey, 10.79925);
    Preferences.initDouble(Keys.drive_kAKey, 0.506595);
    Preferences.initDouble(Keys.auto_kPXKey,10.5);
    Preferences.initDouble(Keys.auto_kPThetaKey, 7.6);
    Preferences.initDouble(Keys.maxSpeedKey, 4.17);
    Preferences.initDouble(Keys.maxAngularVelocityKey, 29.65);

    //Intake, Index, Launch
    Preferences.initDouble(Keys.indexVoltKey, 6.0);
    Preferences.initDouble(Keys.indexAmpVoltKey, 3.0);
    Preferences.initDouble(Keys.intakeVoltKey, 4.5);
    Preferences.initDouble(Keys.speakerHighAimV, 65);
    Preferences.initDouble(Keys.speakerLowAimV, -80);
    Preferences.initDouble(Keys.ampUpperV, 5.0);
    Preferences.initDouble(Keys.ampMiddleV, 8.5);
    Preferences.initDouble(Keys.launcherTolerance, 6);

    Preferences.initBoolean(Keys.characterizationKey, false);

    Preferences.initDouble(Keys.minimumNoteProximityKey, 500);

    //Testing
    Preferences.initDouble(Keys.correctNotePositionKey, 0.85);
    Preferences.initDouble(Keys.indexerkPKey, 2.00);
    Preferences.initDouble(Keys.indexerkIKey, 0.00);
    Preferences.initDouble(Keys.indexerkDKey, 0.20);
  }
}
