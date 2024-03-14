// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.utilities.Constants;

import frc.lib.utilities.LimelightTarget;
import frc.robot.commands.*;
import frc.robot.commands.Index.IndexDirection;
import frc.robot.commands.Launch.LaunchDirection;
import frc.robot.commands.Index;
import frc.robot.commands.Launch;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.utilities.Constants.OperatorConstants;
import frc.robot.commands.Index;
import frc.robot.commands.TeleopDrive;

public class RobotContainer {
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  private final Joystick driver = new Joystick(OperatorConstants.driverControllerPort);
  private final Joystick operator = new Joystick(OperatorConstants.operatorControllerPort);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton launchAmpAutomatic = new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton launchAmpManual = new JoystickButton(operator, XboxController.Button.kBack.value);

  private final JoystickButton launchLowAutomatic = new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton launchLowManual = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton launchHighAutomatic = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton launchHighManual = new JoystickButton(operator, XboxController.Button.kX.value);

  private final JoystickButton indexerIntake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton indexerOuttake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

  private final JoystickButton climber = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
  private final JoystickButton launcher = new JoystickButton(operator, XboxController.Button.kRightStick.value);

  private final SendableChooser<Command> autoChooser;

  private double launcherkV = 0.14;
  private double launcherkp = 8.0;
  private double launcherki = 0.001;
  private double launcherkd = 0.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    registerNamedCommands();

    drivetrainSubsystem.setDefaultCommand(
      new TeleopDrive(
          drivetrainSubsystem,
          () -> -driver.getRawAxis(translationAxis),
          () -> -driver.getRawAxis(strafeAxis),
          () -> -driver.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()
        )
    );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    SmartDashboard.putNumber("V", launcherkV);
    SmartDashboard.putNumber("p", launcherkp);
    SmartDashboard.putNumber("i", launcherki);
    SmartDashboard.putNumber("d", launcherkd);

    SmartDashboard.putNumber("upperSpeakerV", 80);
    SmartDashboard.putNumber("lowerSpeakerV", -80);
    SmartDashboard.putNumber("upperAmpV", 20);

    SmartDashboard.putNumber("indexVolt", 6.0);
    SmartDashboard.putNumber("intakeVolt", 3.0);

    // Configure button bindings
    configureBindings();
  }

  private void configureBindings() {

    //The keybinds and commands for system identification only load if the mode is enabled in constants (for programming purposes).
    if (Constants.SystemToggles.systemIdentification) {
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

    
    //Launches Notes (Automatic launches after spinup, Manual only launches after spinup and button release)
    launchLowAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.LOW, true));
    launchLowManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.LOW, false));
    launchLowManual.onFalse(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.LOW, true).withTimeout(0.75));
    
    launchHighAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.HIGH, true));
    launchHighManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.HIGH, false));
    launchHighManual.onFalse(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.HIGH, true).withTimeout(0.75));
    
    launchAmpAutomatic.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.AMP, true));
    launchAmpManual.whileTrue(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.AMP, false));
    launchAmpManual.onFalse(new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.AMP, true).withTimeout(1.00));
    

    indexerIntake.whileTrue(new Index(indexerSubsystem, IndexDirection.INTAKE_RECKLESS)); //Reckless
    indexerOuttake.whileTrue(new Index(indexerSubsystem, IndexDirection.OUTTAKE));

    climber.whileTrue(new InstantCommand(() -> {
      pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.leftClimber);
      pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.rightClimber);
    }));
    climber.whileFalse(new InstantCommand(() -> {
      pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.leftClimber);
      pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.rightClimber);
    }));

    launcher.whileTrue(new InstantCommand(() -> {
      pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerLeft);
      pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerRight);
    }));
    launcher.whileFalse(new InstantCommand(() -> {
      pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerLeft);
      pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerRight);
    }));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("LaunchNoteLow", new Launch(launcherSubsystem, indexerSubsystem, LaunchDirection.LOW, true).withTimeout(2.5));

    NetworkTable sgs = NetworkTableInstance.getDefault().getTable("sgs");

    boolean isBlue = DriverStation.getAlliance().filter(a -> a == DriverStation.Alliance.Blue).isPresent();
    if (isBlue) {
      // Set targeting parameters for blue
      LimelightTarget target0 = new LimelightTarget(
          7,
          sgs.getEntry("aim_0_tx").getDouble(0.0),
          sgs.getEntry("aim_0_ty").getDouble(0.0),
          sgs.getEntry("aim_0_heading").getDouble(-140.0),
          sgs.getEntry("aim_0_tolerance").getDouble(0.0));
      LimelightTarget target1 = new LimelightTarget(
          7,
          sgs.getEntry("aim_1_tx").getDouble(0.0),
          sgs.getEntry("aim_1_ty").getDouble(0.0),
          sgs.getEntry("aim_1_heading").getDouble(-160.0),
          sgs.getEntry("aim_1_tolerance").getDouble(0.0));

      NamedCommands.registerCommand("Aim-0", new Aim(target0, drivetrainSubsystem));
      NamedCommands.registerCommand("Aim-1", new Aim(target1, drivetrainSubsystem));
    } else {
      // Set targeting parameters for red
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Auto"));
  }
}
