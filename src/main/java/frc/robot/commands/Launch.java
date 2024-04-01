// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj.Joystick;
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
  private final Joystick controller;
  private boolean launchButtonPressed = false;

  public Launch(LauncherSubsystem launcherSubsystem, IndexerSubsystem indexerSubsystem, PneumaticsSubsystem pneumaticsSubsystem, LaunchDirection direction, boolean automatic, Joystick controller) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.launcherSubsystem = launcherSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(launcherSubsystem);

    this.direction = direction;
    this.automatic = automatic;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     switch (direction) {
      case LOW:
        launcherSubsystem.spinMiddleFlywheel(direction);
        launcherSubsystem.spinBottomFlywheel();
        break;
      case HIGH:
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerRight);
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerLeft);
        launcherSubsystem.spinTopFlywheel(direction);
        launcherSubsystem.spinMiddleFlywheel(direction);
        break;
      case AMP:
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerRight);
        pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerLeft);
        launcherSubsystem.spinTopFlywheel(direction);
        launcherSubsystem.spinMiddleFlywheel(direction);
        break;
      default: break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     switch (direction) {
       case LOW:
         launcherSubsystem.spinMiddleFlywheel(direction);
         launcherSubsystem.spinBottomFlywheel();
         break;
       case HIGH:
         pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerRight);
         pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerLeft);
         launcherSubsystem.spinTopFlywheel(direction);
         launcherSubsystem.spinMiddleFlywheel(direction);
         break;
       case AMP:
         pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerRight);
         pneumaticsSubsystem.setSolenoidToForward(pneumaticsSubsystem.noteAimerLeft);
         launcherSubsystem.spinTopFlywheel(direction);
         launcherSubsystem.spinMiddleFlywheel(direction);
         break;
       default: break;
     }
     */

    if (controller.getPOV() < 45 || controller.getPOV() > 315) {
      launchButtonPressed = true;
    }
    
    //Need to decide if pressing 'up' should immediately fire note regardless of speed or if accuarcey is more important.
    //Moves the note into the flywheels if the launchers are up to speed. Automatic mode just does this as soon as possible. Manual mode requires pressing 'up' on the dpad before occuring.
    if (launcherSubsystem.isLauncherUpToSpeed(direction)) {
      if (automatic || launchButtonPressed) {
        indexerSubsystem.indexNote(direction == LaunchDirection.AMP);
      }
    }

  }

  @Override
  public void end(boolean interrupted) {

    launcherSubsystem.stopSpinners();
    indexerSubsystem.stopIndexer();
    pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerRight);
    pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerLeft);
  }
}

