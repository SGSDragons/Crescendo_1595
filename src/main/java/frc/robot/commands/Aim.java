package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.LimelightTarget;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Aim extends Command {

    public final LimelightTarget target;
    public DrivetrainSubsystem drivetrainSubsystem;

    // These values will be used unless defined in NT's aim parameters
    public final double defaultXGain = 0.0;
    public final double defaultYGain = 0.0;
    public final double defaultHeadingGain = 0.0;

    NetworkTableEntry xGain;
    NetworkTableEntry yGain;
    NetworkTableEntry headingGain;

    public Aim(LimelightTarget target, DrivetrainSubsystem drivetrainSubsystem) {
        this.target = target;
        this.drivetrainSubsystem = drivetrainSubsystem;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("alignment_pid");
        xGain = table.getEntry("x_gain");
        yGain = table.getEntry("y_gain");
        headingGain = table.getEntry("heading_gain");
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    public void initialize() {}

    /** The main body of a command. Called repeatedly while the command is scheduled. */
    public void execute() {
        // has the camera always check the april tag id, tx, and ty
        //   Which ID? What are the right values for tx and ty?

        // take the tx and ty and then devide them by themselves so that at first the power is at one and as the robot
        // gets closer, the power goes down with it stopping when the power gets low enough.

        LimelightTarget.Error error = target.find(drivetrainSubsystem.getHeading().getDegrees());

        if (error == null) {
            // What to do when the april tag isn't visible?
            return;
        }

        double strafeValue = error.x * xGain.getDouble(defaultXGain);
        double translationValue = error.y * yGain.getDouble(defaultYGain);
        double dTheta = error.heading * headingGain.getDouble(defaultHeadingGain);

        drivetrainSubsystem.drive(
                new Translation2d(translationValue, strafeValue),
                dTheta,
                false,
                false);
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     *
     * <p>Do not schedule commands here that share requirements with this command. Use {@link
     * #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    public void end(boolean interrupted) {
        // turns off all motors?
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    public boolean isFinished() {
        LimelightTarget.Error error = target.find(drivetrainSubsystem.getHeading().getDegrees());

        if (error == null) {
            // Keep trying?
            return false;
        }

        return error.x == 0 && error.y == 0 && error.heading == 0;
    }

}
