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
    public final double tolerance;
    public DrivetrainSubsystem drivetrainSubsystem;


    public static final NetworkTableEntry xGain;
    public static final NetworkTableEntry yGain;
    public static final NetworkTableEntry headingGain;
    static {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("aim_pid");
        xGain = table.getEntry("x_gain");
        yGain = table.getEntry("y_gain");
        headingGain = table.getEntry("heading_gain");

        // These values will be used unless edited in NT's aim parameters
        // Find good values in testing, then hard code them here.
        xGain.setDouble(0.0);
        yGain.setDouble(0.0);
        headingGain.setDouble(0.0);
    }


    public Aim(LimelightTarget target, double tolerance, DrivetrainSubsystem drivetrainSubsystem) {
        this.target = target;
        this.tolerance = tolerance;
        this.drivetrainSubsystem = drivetrainSubsystem;
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

        double strafeValue = error.x * xGain.getDouble(0.0);
        double translationValue = error.y * yGain.getDouble(0.0);
        double dTheta = error.heading * headingGain.getDouble(0.0);

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

    int misses = 0;
    public boolean isFinished() {
        LimelightTarget.Error error = target.find(drivetrainSubsystem.getHeading().getDegrees());

        if (error == null) {
            misses += 1;
            // Give up if there are more than 20 checks and the tag isn't found.
            return misses > 20;
        }

        misses = 0;
        return Math.abs(error.x) < tolerance && Math.abs(error.y) < tolerance && error.heading == 3;
    }
}
