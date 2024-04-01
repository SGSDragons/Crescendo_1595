// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.lib.utilities.LimelightHelpers;
import frc.robot.subsystems.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class Index extends Command {

  public enum IndexDirection {
    INTAKE,
    OUTTAKE,
  }

  private final IndexerSubsystem indexerSubsystem;
  private final IndexDirection direction;

  public Index(IndexerSubsystem indexerSubsystem, IndexDirection direction) {
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(indexerSubsystem);

    this.direction = direction;
  }


  @Override //Test if on Initialization is better overall:
  public void initialize() {
    switch (direction) {
      case INTAKE:
        indexerSubsystem.intakeNote();
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

    if (indexerSubsystem.isNoteLoaded()) {
      LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

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
      LimelightHelpers.setLEDMode_ForceOff("limelight");
  }
}

