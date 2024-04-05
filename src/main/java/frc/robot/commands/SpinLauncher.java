// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.LaunchTeleop.LaunchDirection;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SpinLauncher extends Command {

  private final LauncherSubsystem launcherSubsystem;
  private final PneumaticsSubsystem pneumaticsSubsystem;
  private final LaunchDirection direction;

  public SpinLauncher(LauncherSubsystem launcherSubsystem, PneumaticsSubsystem pneumaticsSubsystem, LaunchDirection direction) {
    this.pneumaticsSubsystem = pneumaticsSubsystem;
    this.launcherSubsystem = launcherSubsystem;
    addRequirements(launcherSubsystem);

    this.direction = direction;
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

  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.stopSpinners();
    pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerRight);
    pneumaticsSubsystem.setSolenoidToReverse(pneumaticsSubsystem.noteAimerLeft);
  }
}

