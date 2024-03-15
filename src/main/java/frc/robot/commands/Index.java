// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class Index extends Command {

  public enum IndexDirection {
    INTAKE,
    OUTTAKE,
    INTAKE_DISREGARD_LOADING // Intakes without stopping when note is loaded.
  }

  private final IndexerSubsystem indexerSubsystem;
  private final IndexDirection direction;

  public Index(IndexerSubsystem indexerSubsystem, IndexDirection direction) {
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(indexerSubsystem);

    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (direction) {
      case INTAKE:
        indexerSubsystem.indexNoteIntake();
        break;
      case OUTTAKE:
        indexerSubsystem.indexNoteOuttake();
        break;
      case INTAKE_DISREGARD_LOADING:
        indexerSubsystem.indexNoteIntakeDisregardLoading();
        break;
      default: break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

