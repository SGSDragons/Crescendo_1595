// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;

import frc.lib.utilities.LimelightTarget;
import frc.robot.commands.*;
import frc.robot.commands.Index.IndexDirection;
import frc.robot.commands.LaunchTeleop.LaunchDirection;
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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.utilities.Constants.OperatorConstants;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
  public final static boolean isBlue = isBlue();

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
  private final JoystickButton slowTurn = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

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
          () -> robotCentric.getAsBoolean()
          ));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {

    //Keybinds for... actually driving the robot in TeleOP.
    zeroGyro.onTrue(new InstantCommand(() -> drivetrainSubsystem.zeroHeading()));

    slowTurn.whileTrue(new TeleopDrive(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> -driver.getRawAxis(rotationAxis) * 0.25,
          () -> robotCentric.getAsBoolean()
          ));

    autoAim.whileTrue(new TeleopAim(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> -driver.getRawAxis(rotationAxis) * 0.25,
          () -> robotCentric.getAsBoolean(),
          (rumbleIntensity) -> {
            driver.setRumble(GenericHID.RumbleType.kBothRumble, rumbleIntensity);
            operator.setRumble(GenericHID.RumbleType.kBothRumble, rumbleIntensity);
          }
          ));

    //Launches Notes (Automatic launches after spinup, Manual only launches after spinup and 'up' on dpad)
    launchLowAutomatic.whileTrue(new LaunchTeleop(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, true, operator));
    launchLowManual.whileTrue(new LaunchTeleop(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.LOW, false, operator));
    
    launchHighAutomatic.whileTrue(new LaunchTeleop(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, true, operator));
    launchHighManual.whileTrue(new LaunchTeleop(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH, false, operator));
    
    launchAmpAutomatic.whileTrue(new LaunchTeleop(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, true, operator));
    launchAmpManual.whileTrue(new LaunchTeleop(launcherSubsystem, indexerSubsystem, pneumaticsSubsystem, LaunchDirection.AMP, false, operator));
    

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

  // private boolean isControllerRightAxisPressed (Joystick controller) {
  //   return controller.getRawAxis(3) > 0.25;
  // }
  
  private void registerNamedCommands() {
    NamedCommands.registerCommand("LaunchNoteLow", new ParallelDeadlineGroup(new SpinLauncher(launcherSubsystem, pneumaticsSubsystem, LaunchDirection.LOW).withTimeout(1.0), new WaitCommand(0.5).andThen(new Index(indexerSubsystem, launcherSubsystem, IndexDirection.LAUNCH))));
    NamedCommands.registerCommand("LaunchNoteHigh", new ParallelDeadlineGroup(new SpinLauncher(launcherSubsystem, pneumaticsSubsystem, LaunchDirection.HIGH).withTimeout(1.0), new WaitCommand(0.5).andThen(new Index(indexerSubsystem, launcherSubsystem, IndexDirection.LAUNCH))));
    NamedCommands.registerCommand("LaunchNoteAmp", new ParallelDeadlineGroup(new SpinLauncher(launcherSubsystem, pneumaticsSubsystem, LaunchDirection.AMP).withTimeout(1.0), new WaitCommand(0.5).andThen(new Index(indexerSubsystem, launcherSubsystem, IndexDirection.LAUNCH))));
    
    NamedCommands.registerCommand("Intake",
      new Index(indexerSubsystem, launcherSubsystem, IndexDirection.INTAKE).withTimeout(1.25));

    NamedCommands.registerCommand("IntakeLong",
      new Index(indexerSubsystem, launcherSubsystem, IndexDirection.INTAKE).withTimeout(4.0));

    NamedCommands.registerCommand("CorrectNotePosition", new CorrectNotePosition(indexerSubsystem).withTimeout(0.5));

    //Auto Aim Commands
    List<LimelightTarget> targets = isBlue ? blueTargets() : redTargets();
    for (int i=0; i < targets.size(); ++i) {
      NamedCommands.registerCommand("aim-"+i, new AutoAim(targets.get(i), drivetrainSubsystem).withTimeout(0.75));
    }
  }

  private static boolean isBlue() {
    return DriverStation.getAlliance().filter(a -> a == DriverStation.Alliance.Blue).isPresent();
  }

  public Command getAutonomousCommand() {
    //drivetrainSubsystem.zeroHeading();
    return autoChooser.getSelected();
  }

  public List<LimelightTarget> blueTargets() {
    List<LimelightTarget> targets = new ArrayList<>();

    targets.add(new LimelightTarget(
        7,
        // -5.385597,
        0,
        7.125707,
        -12.60));
    return targets;
  }

  public List<LimelightTarget> redTargets() {
    List<LimelightTarget> targets = new ArrayList<>();

    targets.add(new LimelightTarget(
            4,
            // 5.385597,
            0,
            7.125707,
            12.60));
    return targets;
  }

  //Network Tables Telemetry, KEEP OFF DURING MATHCES ON FIELD TO CONSERVE BANDWIDTH
  public void updateLLTargetTelemetry() {
    NetworkTableEntry focusT = NetworkTableInstance.getDefault().getTable("sgs").getEntry("ll_target");
    int focus = (int)focusT.getInteger(0);
    focusT.setInteger(focus);

    List<LimelightTarget> targets = isBlue() ? blueTargets() : redTargets();
    if (focus >= 0 && focus < targets.size()) {
      targets.get(focus).find(drivetrainSubsystem.getHeading().getDegrees());
    }
  } 
   

  //Some of these values will be phased out for performance enhancement.
  public void initializeRobotPreferences() {
    // Driving
    Preferences.initDouble(Keys.auto_kPXKey,10.5);
    Preferences.initDouble(Keys.auto_kPThetaKey, 7.6);
    Preferences.initDouble(Keys.maxSpeedKey, 4.17);
    Preferences.initDouble(Keys.maxAngularVelocityKey, 29.65);

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
