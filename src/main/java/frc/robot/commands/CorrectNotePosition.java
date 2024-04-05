package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class CorrectNotePosition extends Command{
    
    private final IndexerSubsystem indexerSubsystem;
    private double targetPosition;

    public CorrectNotePosition(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        targetPosition = indexerSubsystem.setTargetPosition();
    }

    @Override
    public void execute() {
        indexerSubsystem.correctNotePosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return indexerSubsystem.isNoteCorrectlyPositioned(targetPosition);
    }

}
