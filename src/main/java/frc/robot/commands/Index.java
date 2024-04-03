// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Index extends Command {

  public enum IndexDirection {
    INTAKE,
    OUTTAKE,
  }

  private final IndexerSubsystem indexerSubsystem;
  private final IndexDirection direction;
  private final LauncherSubsystem launcherSubsystem;

  public Index(IndexerSubsystem indexerSubsystem, LauncherSubsystem launcherSubsystem, IndexDirection direction) {
    this.indexerSubsystem = indexerSubsystem;
    this.launcherSubsystem = launcherSubsystem;
    addRequirements(indexerSubsystem);

    this.direction = direction;
  }


  @Override //Test if on Initialization is better overall:
  public void initialize() {
    switch (direction) {
      case INTAKE:
        indexerSubsystem.intakeNote();
        launcherSubsystem.spinBottomFlywheel(2);
        launcherSubsystem.spinMiddleFlywheel(2);
        break;
      case OUTTAKE:
        indexerSubsystem.outtakeNote();
        break;
      default:
        break;
    }
  }


  @Override
  public void execute() {
    /*
     * 
     if (indexerSubsystem.isNoteLoaded()) {
       LimelightHelpers.setLEDMode_ForceOn("limelight");
     }
     */


/*
 switch (direction) {
   case INTAKE:
     indexerSubsystem.intakeNote();
     break;
   case OUTTAKE:
     indexerSubsystem.outtakeNote();
     break;
   default: break;
 }
 */
  }

  @Override
  public void end(boolean interrupted) {
      indexerSubsystem.stopIndexer();
      launcherSubsystem.stopSpinners();
      //LimelightHelpers.setLEDMode_ForceOff("limelight");
  }
}

