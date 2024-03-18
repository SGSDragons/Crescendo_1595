package frc.lib.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightTarget {

    public static class Error {
        public final double x;
        public final double y;
        public final double heading;

        public Error(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public final int tagId;
    public final double wantedX;
    public final double wantedY;
    public final double wantedHeading;

    public final double lateralTolerance;

    private final NetworkTableEntry errorX;
    private final NetworkTableEntry errorY;
    private final NetworkTableEntry errorHeading;
    private final NetworkTableEntry tagVisible;
    private final NetworkTableEntry locked;

    public LimelightTarget(int tagId, double wantedX, double wantedY, double wantedHeading, double lateralTolerance) {
        this.tagId = tagId;
        this.wantedX = wantedX;
        this.wantedY = wantedY;
        this.wantedHeading = wantedHeading;
        this.lateralTolerance = lateralTolerance;

        NetworkTable table = NetworkTableInstance.getDefault().getTable("ll_target");
        table.getEntry("target_id").setInteger(tagId);

        tagVisible = table.getEntry("visible");
        errorX = table.getEntry("err_x");
        errorY = table.getEntry("err_y");
        errorHeading = table.getEntry("err_heading");
        locked = table.getEntry("locked");
    }

    public Error find(double heading) {
        LimelightHelpers.LimelightTarget_Fiducial[] aprilTags = LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Fiducials;

        for (LimelightHelpers.LimelightTarget_Fiducial aprilTag: aprilTags) {
            if (aprilTag.fiducialID == tagId) {
                Error error = new Error(
                    MathUtil.applyDeadband(aprilTag.tx - wantedX, lateralTolerance),
                    MathUtil.applyDeadband(aprilTag.ty - wantedY, lateralTolerance),
                    MathUtil.applyDeadband(heading - wantedHeading, 3)
                );
                errorX.setDouble(error.x);
                errorY.setDouble(error.y);
                errorHeading.setDouble(error.heading);
                tagVisible.setBoolean(true);
                locked.setBoolean(error.x == 0 && error.y == 0 && error.heading == 0);
                return error;
            }
        }

        errorX.setDouble(0);
        errorY.setDouble(0);
        errorHeading.setDouble(0);
        tagVisible.setBoolean(false);
        return null;

    }
}
