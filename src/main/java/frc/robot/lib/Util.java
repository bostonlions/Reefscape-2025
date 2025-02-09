package frc.robot.lib;

public final class Util {
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-12);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double minResult = scopeReference - 180;
        return placeIn0To360Scope(newAngle - minResult) + minResult;
    }

    public static double placeIn0To360Scope(double newAngle){
        return (newAngle % 360. + 360.) % 360.;
    }

    public static double deadBand(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.;
    }

    public static double scaledDeadband(double value, double maxValue, double deadband) {
        double deadbandedValue = deadBand(value, deadband);
        if (epsilonEquals(deadbandedValue, 0.)) return 0.;
        return Math.signum(deadbandedValue) * ((Math.abs(deadbandedValue) - deadband) / (maxValue - deadband));
    }

    public final class Conversions {
        public static double rotationsToDegrees(double rotations, double gearRatio) {
            return rotations * 360. / gearRatio;
        }

        public static double degreesToRotation(double degrees, double gearRatio) {
            return degrees * gearRatio / 360.;
        }

        public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
            return velocity * gearRatio / circumference;
        }

        public static double RPSToMPS(double rotationsPerSecond, double circumference, double gearRatio) {
            return rotationsPerSecond * circumference / gearRatio;
        }

        public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
            return rotations * circumference / gearRatio;
        }

        public static double metersToRotations(double meters, double circumference, double gearRatio) {
            return meters * gearRatio / circumference;
        }

        public static double metersToInches(double meters) {
            return meters / 0.0254;
        }

        public static double inchesToMeters(double inches) {
            return inches * 0.0254;
        }
    }
}
