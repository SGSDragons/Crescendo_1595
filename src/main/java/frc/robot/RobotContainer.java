// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.utilities.Constants.OperatorConstants;
import frc.lib.utilities.Constants.SystemToggles;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
  private static boolean compressorOnly;

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

  private final JoystickButton autoAim = new JoystickButton(driver, XboxController.Button.kX.value);
  
  private final JoystickButton compressor = new JoystickButton(driver, XboxController.Button.kX.value);

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeRobotPreferences();
    registerNamedCommands();

    compressorOnly = Preferences.getBoolean(Keys.compressorOnlyKey, false);

    if (!compressorOnly) {
      drivetrainSubsystem.setDefaultCommand(
        new TeleopDrive(
            drivetrainSubsystem,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()
          )
      );
    }

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
    if (compressorOnly) {
      pneumaticsSubsystem.enableCompressor();
      return;
    }

    zeroGyro.onTrue(new InstantCommand(() -> drivetrainSubsystem.zeroHeading()));

    slowAim.whileTrue(new TeleopDrive(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> -driver.getRawAxis(rotationAxis) * 0.25,
          () -> robotCentric.getAsBoolean()
        ));

    //Launches Notes (Automatic launches after spinup, Manual only launches after spinup and button release)
    launchLowAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, true));
    launchLowManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, false));
    launchLowManual.onFalse(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, true).withTimeout(0.75));
    
    launchHighAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, true));
    launchHighManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, false));
    launchHighManual.onFalse(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, true).withTimeout(0.75));
    
    launchAmpAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, true));
    launchAmpManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, false));
    launchAmpManual.onFalse(new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, true).withTimeout(1.00));
    

    indexerIntake.whileTrue(new Index(indexerSubsystem, IndexDirection.INTAKE_DISREGARD_LOADING)); //Does not check for if note is loaded
    indexerOuttake.whileTrue(new Index(indexerSubsystem, IndexDirection.OUTTAKE));

    //autoAim.whileTrue(new Aim(blueTargets().get(0), drivetrainSubsystem));
    int speakerTag = isBlue() ? 7 : 4;
    autoAim.whileTrue(new TwistAim(
            new LimelightTarget(speakerTag, 0, 0, 0),
            drivetrainSubsystem
          ));

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
    NamedCommands.registerCommand("LaunchNoteLow", new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, true).withTimeout(1.5));
    NamedCommands.registerCommand("LaunchNoteHigh", new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, true).withTimeout(1.5));
    NamedCommands.registerCommand("LaunchNoteAmp", new Launch(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, true).withTimeout(1.5));
    NamedCommands.registerCommand("Intake",
      new Index(indexerSubsystem, IndexDirection.INTAKE).withTimeout(2.0).andThen(new Index(indexerSubsystem, IndexDirection.OUTTAKE).withTimeout(0.1))); ///Isn't able to detect when to stop, so need to outtake by some arbitrary amount to get note in right position.

    NamedCommands.registerCommand("IntakeLong",
      new Index(indexerSubsystem, IndexDirection.INTAKE).withTimeout(7.7).andThen(new Index(indexerSubsystem, IndexDirection.OUTTAKE).withTimeout(0.1))); ///Isn't able to detect when to stop, so need to outtake by some arbitrary amount to get note in right position.

    NetworkTable sgs = NetworkTableInstance.getDefault().getTable("sgs");

    boolean isBlue = isBlue();
    List<LimelightTarget> targets = isBlue ? blueTargets() : redTargets();

    for (int i=0; i < targets.size(); ++i) {
      // Play with tolerance until we are happy it gets close enough fast enough.
      NamedCommands.registerCommand("aim-"+i, new Aim(targets.get(i), 0.5, drivetrainSubsystem));
    }
  }

  private static boolean isBlue() {
    return DriverStation.getAlliance().filter(a -> a == DriverStation.Alliance.Blue).isPresent();
  }

  public Command getAutonomousCommand() {
    drivetrainSubsystem.zeroHeading();
    if (SystemToggles.useCompleteAuto) {
      boolean isBlue = isBlue();
      String pathFile = isBlue ? "Auto-Blue" : "Auto-Red";
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathFile));
    }
    else {
      return autoChooser.getSelected();
    }
  }

  public List<LimelightTarget> blueTargets() {
    NetworkTable sgs = NetworkTableInstance.getDefault().getTable("sgs");
    List<LimelightTarget> targets = new ArrayList<>();

    targets.add(new LimelightTarget(
        7,
        sgs.getEntry("aim_0_tx").getDouble(0.0),
        sgs.getEntry("aim_0_ty").getDouble(0.0),
        sgs.getEntry("aim_0_heading").getDouble(-150.0)));
    targets.add(new LimelightTarget(
        7,
        sgs.getEntry("aim_1_tx").getDouble(0.0),
        sgs.getEntry("aim_1_ty").getDouble(0.0),
        sgs.getEntry("aim_1_heading").getDouble(-160.0)));
    targets.add(new LimelightTarget(
        7,
        sgs.getEntry("aim_2_tx").getDouble(0.0),
        sgs.getEntry("aim_2_ty").getDouble(0.0),
        sgs.getEntry("aim_2_heading").getDouble(-180.0)));

    return targets;
  }

  public List<LimelightTarget> redTargets() {
    NetworkTable sgs = NetworkTableInstance.getDefault().getTable("sgs");
    List<LimelightTarget> targets = new ArrayList<>(); //Is it planned for this to have the same targets but with a different tag or something completely different?

    targets.add(new LimelightTarget(
            4,
            sgs.getEntry("aim_0_tx").getDouble(-12.0),
            sgs.getEntry("aim_0_ty").getDouble(8.0),
            sgs.getEntry("aim_0_heading").getDouble(0.0)));
    targets.add(new LimelightTarget(
            4,
            sgs.getEntry("aim_1_tx").getDouble(0.0),
            sgs.getEntry("aim_1_ty").getDouble(0.0),
            sgs.getEntry("aim_1_heading").getDouble(-25.0)));
    targets.add(new LimelightTarget(
            4,
            sgs.getEntry("aim_2_tx").getDouble(0.0),
            sgs.getEntry("aim_2_ty").getDouble(0.0),
            sgs.getEntry("aim_2_heading").getDouble(0.0)));

    return targets;
  }

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
    Preferences.initDouble(Keys.drive_kSKey, 0.32);
    Preferences.initDouble(Keys.drive_kVKey, 1.51);
    Preferences.initDouble(Keys.drive_kAKey, 0.27);
    Preferences.initDouble(Keys.auto_kPXKey, 3);
    Preferences.initDouble(Keys.auto_kPThetaKey, 4);
    Preferences.initDouble(Keys.maxSpeedKey, 4.17);
    Preferences.initDouble(Keys.maxAngularVelocityKey, 29.65);

    //Intake, Index, Launch
    Preferences.initDouble(Keys.indexVoltKey, 6.0);
    Preferences.initDouble(Keys.indexAmpVoltKey, 1.5);
    Preferences.initDouble(Keys.intakeVoltKey, 3.0);
    Preferences.initDouble(Keys.speakerHighAimV, 80);
    Preferences.initDouble(Keys.speakerLowAimV, -80);
    Preferences.initDouble(Keys.ampV, 40);
    Preferences.initDouble(Keys.launcherTolerance, 15);

    Preferences.initBoolean(Keys.characterizationKey, false);
    Preferences.initBoolean(Keys.compressorOnlyKey, false);


  }
}
