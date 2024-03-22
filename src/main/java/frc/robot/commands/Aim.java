package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utilities.LimelightHelpers;
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
        xGain.setDouble(0.1);
        yGain.setDouble(0.1);
        headingGain.setDouble(0.01);
    }


    public Aim(LimelightTarget target, double tolerance, DrivetrainSubsystem drivetrainSubsystem) {
        this.target = target;
        this.tolerance = tolerance;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

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

        double x = MathUtil.applyDeadband(error.x, tolerance);
        double y = MathUtil.applyDeadband(error.y, tolerance);
        double heading = MathUtil.applyDeadband(error.heading, tolerance);
        double strafeValue = x * xGain.getDouble(0.0);
        double translationValue = y * yGain.getDouble(0.0);
        double dTheta = heading * headingGain.getDouble(0.0);

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
        LimelightHelpers.setLEDMode_ForceOff("limelight");
        ChassisSpeeds stop = new ChassisSpeeds(0, 0, 0);
        drivetrainSubsystem.drive(stop);
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
            boolean fail = misses > 100;
            if (fail) {
                DriverStation.reportWarning("Aim Giving up", false);
            }
            return fail;
        }


        misses = 0;
        boolean locked = Math.abs(error.x) < tolerance && Math.abs(error.y) < tolerance && Math.abs(error.heading) < 3;
        if (locked) {
            DriverStation.reportWarning("Aim Complete", false);
        }
        return locked;
    }
}
