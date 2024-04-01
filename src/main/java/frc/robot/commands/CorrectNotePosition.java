package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class CorrectNotePosition extends Command{
    
    private final IndexerSubsystem indexerSubsystem;

    public CorrectNotePosition(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.correctNotePosition();
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return indexerSubsystem.isNoteCorrectlyPositioned();
    }

}
