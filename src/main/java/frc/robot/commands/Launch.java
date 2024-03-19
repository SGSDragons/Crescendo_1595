// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Launch extends Command {

  public enum LaunchDirection {
    LOW,
    HIGH,
    AMP //If Amp Requires it
  }

  private final LauncherSubsystem launcherSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final PneumaticsSubsystem pneumaticsSubsystem;
  private final LaunchDirection direction;
  private final boolean automatic;

  public Launch(LauncherSubsystem launcherSubsystem, IndexerSubsystem indexerSubsystem, PneumaticsSubsystem pneumaticsSubsystem, LaunchDirection direction, boolean automatic) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.launcherSubsystem = launcherSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(launcherSubsystem);

    this.direction = direction;
    this.automatic = automatic;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (direction) {
      case LOW:
        launcherSubsystem.spinLowerSpinners();
        break;
      case HIGH:
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerRight);
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerLeft);
        launcherSubsystem.spinUpperSpinners(false);
        break;
      case AMP:
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerRight);
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerLeft);
        launcherSubsystem.spinUpperSpinners(true);
        break;
      default: break;
    }
    
    if (launcherSubsystem.isLauncherUpToSpeed(direction) && automatic) {
      indexerSubsystem.indexNoteLaunchSpeaker(direction == LaunchDirection.AMP);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!automatic) {
      return;
    }

    launcherSubsystem.stopSpinners();
    indexerSubsystem.stopIndexer();
    pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerRight);
    pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerLeft);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

