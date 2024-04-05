package frc.lib.utilities;

public final class Utilities {

    public static double rangeClamp(double value, double maximum, double minimum) {
        if (maximum > value && value > minimum) {
            return value;
        }
        if (maximum < value) {
            return maximum;
        }
        return minimum;
    }
}
