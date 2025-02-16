package frc.robot.lib;

public final class Util {
    public static double limit(double v, double maxMagnitude) {
        return Math.min(maxMagnitude, Math.max(-maxMagnitude, v));
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double minResult = scopeReference - 180;
        return placeIn0To360Scope(newAngle - minResult) + minResult;
    }

    public static double placeIn0To360Scope(double newAngle) {
        return (newAngle % 360. + 360.) % 360.;
    }

    public static final class Conversions {
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
    }
}
