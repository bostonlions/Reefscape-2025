package frc.robot.lib;

import java.util.List;

/** Contains basic functions that are used often. */
public class Util {
    public static final double kEpsilon = 1e-12;

    private Util() {};

    /** Limits @param v to @param maxMagnitude */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /** Checks if @param v is within the range @param min to @param max both exclusive. */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double clamp(double value, double min, double max) {
		if (min > max) {
			throw new IllegalArgumentException("min must not be greater than max");
		}
		return Math.max(min, Math.min(value, max));
	}

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public static double placeIn0To360Scope(double newAngle){
        double lowerBound = 0;
        double upperBound = 360;
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        return newAngle;
    }

    public static double deadBand(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    public static double scaledDeadband(double value, double maxValue, double deadband) {
        double deadbandedValue = deadBand(value, deadband);
        if (epsilonEquals(deadbandedValue, 0.0))
            return 0.0;
        return Math.signum(deadbandedValue) * ((Math.abs(deadbandedValue) - deadband) / (maxValue - deadband));
    }

    public final class Conversions {
        private Conversions() {};

        // /**
        //  * @param counts Falcon Counts
        //  * @param gearRatio Gear Ratio between Falcon and Mechanism
        //  * @return Degrees of Rotation of Mechanism
        //  */
        // public static double falconToDegrees(double counts, double gearRatio) {
        //     return counts * (360.0 / (gearRatio * 2048.0));
        // }

        // /**
        //  * @param degrees Degrees of rotation of Mechanism
        //  * @param gearRatio Gear Ratio between Falcon and Mechanism
        //  * @return Falcon Counts
        //  */
        // public static double degreesToFalcon(double degrees, double gearRatio) {
        //     double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        //     return ticks;
        // }

        /**
         * @param counts    Falcon Counts
         * @param gearRatio Gear Ratio between Falcon and Mechanism
         * @return Degrees of Rotation of Mechanism
         */
        public static double rotationsToDegrees(double rotations, double gearRatio) {
            return (rotations/gearRatio) * (360.0); //This was changed from citrus
        }

        /**
         * @param degrees   Degrees of rotation of Mechanism
         * @param gearRatio Gear Ratio between Falcon and Mechanism
         * @return
         */
        public static double degreesToRotation(double degrees, double gearRatio) {
            double ticks = degrees / (360.0 / (gearRatio));
            return ticks;
        }

        // /**
        //  * @param velocityCounts Falcon Velocity Counts
        //  * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
        //  * @return RPM of Mechanism
        //  */
        // public static double falconToRPM(double velocityCounts, double gearRatio) {
        //     double motorRPM = velocityCounts * (600.0 / 2048.0);
        //     double mechRPM = motorRPM / gearRatio;
        //     return mechRPM;
        // }

        /**
         * @param RPM RPM of mechanism
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
         * @return RPM of Mechanism
         */
        // public static double RPMToFalcon(double RPM, double gearRatio) {
        //     double motorRPM = RPM * gearRatio;
        //     double sensorCounts = motorRPM * (2048.0 / 600.0);
        //     return sensorCounts;
        // }

        // /**
        //  * @param velocitycounts Falcon Velocity Counts
        //  * @param circumference Circumference of Wheel
        //  * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
        //  * @return Falcon Velocity Counts
        //  */
        // public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        //     double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        //     double wheelMPS = (wheelRPM * circumference) / 60;
        //     return wheelMPS;
        // }

        // /**
        //  * @param velocity Velocity MPS
        //  * @param circumference Circumference of Wheel
        //  * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
        //  * @return Falcon Velocity Counts
        //  */
        // public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        //     double wheelRPM = ((velocity * 60) / circumference);
        //     double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        //     return wheelVelocity;
        // }

        public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
            double wheelRPS = ((velocity) / circumference);
            double falconRPS = wheelRPS * gearRatio;
            return falconRPS;
        }

        public static double RPSToMPS(double rotationsPerSecond, double circumference, double gearRatio) {
            double wheelRPM = (rotationsPerSecond / gearRatio);
            double mps = wheelRPM * circumference;
            return mps;
        }

        // public static double falconToMeters(double falconTicks, double circumference, double gearRatio){
        //     double wheelRevolutions = falconToDegrees(falconTicks, gearRatio) / 360.0;
        //     double wheelDistance = wheelRevolutions * circumference;
        //     return wheelDistance;
        // }

        public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
            double wheelRevolutions = rotations / gearRatio;
            double wheelDistance = wheelRevolutions * circumference;
            return wheelDistance;
        }

        public static double metersToRotations(double meters, double circumference, double gearRatio) {
            double wheelRevolutions = meters / circumference;
            double rotations = wheelRevolutions * gearRatio;
            return rotations;
        }

        // Convert meters to inches
        public static double metersToInches(double meters) {
            return meters * (39.73701 / 1);
        }

        // Convert meters to inches
        public static double inchesToMeters(double inches) {
            return inches * (0.0254 / 1);
        }
    }
}
