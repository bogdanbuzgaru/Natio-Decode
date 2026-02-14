package org.firstinspires.ftc.teamcode.math;



public class ShooterCalculations {

    // Constants
    private static final double GRAVITY = 386.1; // in/s^2 (9.8 m/s^2 converted)

    // Hood servo calibration (you'll need to measure these for your robot)
    private double minHoodAngle;  // degrees
    private double maxHoodAngle;  // degrees
    private double minServoPosition; // servo position at min angle
    private double maxServoPosition; // servo position at max angle

    // Flywheel calibration coefficients (from Excel fitting)
    // flywheelSpeed = a * launchSpeed + b
    private double flywheelSpeedSlope;     // coefficient 'a'
    private double flywheelSpeedIntercept;  // coefficient 'b'

    /**
     * Constructor with calibration parameters
     */
    public ShooterCalculations(double minHoodAngle, double maxHoodAngle,
                               double minServoPosition, double maxServoPosition,
                               double flywheelSpeedSlope, double flywheelSpeedIntercept) {
        this.minHoodAngle = minHoodAngle;
        this.maxHoodAngle = maxHoodAngle;
        this.minServoPosition = minServoPosition;
        this.maxServoPosition = maxServoPosition;
        this.flywheelSpeedSlope = flywheelSpeedSlope;
        this.flywheelSpeedIntercept = flywheelSpeedIntercept;
    }

    /**
     * Result class for shooting parameters
     */
    public static class ShootingParameters {
        public double launchAngle;         // radians
        public double launchSpeed;         // in/s
        public double hoodServoPosition;   // servo position for hood
        public double flywheelSpeed;       // speed for both flywheel motors
        public double distanceToGoal;      // calculated distance (for telemetry)

        public double getLaunchAngleDegrees() {
            return Math.toDegrees(launchAngle);
        }
    }

    // ========================================================================
    // MAIN CALCULATION METHOD
    // ========================================================================

    public ShootingParameters calculateShootingParameters(
            double robotX,
            double robotY,
            double robotVelocityX,
            double robotVelocityY,
            double goalX,
            double goalY,
            double goalHeight,
            double goalEntryAngle,
            double minClampAngle,
            double maxClampAngle) {

        // Calculate distance to goal
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distanceToGoal = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.atan2(dy, dx);

        // Step 1: Get initial launch parameters (stationary robot)
        double alpha = Math.atan((2.0 * goalHeight / distanceToGoal) - Math.tan(goalEntryAngle));

        double cosAlpha = Math.cos(alpha);
        double tanAlpha = Math.tan(alpha);
        double numerator = GRAVITY * distanceToGoal * distanceToGoal;
        double denominator = 2.0 * cosAlpha * cosAlpha * (distanceToGoal * tanAlpha - goalHeight);
        double v0 = Math.sqrt(numerator / denominator);

        // Step 2: Calculate robot velocity magnitude and direction
        double robotSpeed = Math.sqrt(robotVelocityX * robotVelocityX +
                robotVelocityY * robotVelocityY);
        double robotVelocityAngle = Math.atan2(robotVelocityY, robotVelocityX);

        // Step 3: Calculate robot velocity components relative to goal
        double deltaAngle = robotVelocityAngle - angleToGoal;
        double vRadial = -Math.cos(deltaAngle) * robotSpeed;
        double vTangential = Math.sin(deltaAngle) * robotSpeed;

        // Step 4: Calculate time for ball to reach goal
        double vx = v0 * Math.cos(alpha);
        double timeToGoal = distanceToGoal / vx;

        // Step 5: Calculate new ball velocity compensating for robot motion
        double vxCompensatedRadial = vx + vRadial;
        double vxNew = Math.sqrt(vxCompensatedRadial * vxCompensatedRadial +
                vTangential * vTangential);

        // Step 6: Calculate vertical velocity component (unchanged)
        double vy = v0 * Math.sin(alpha);

        // Step 7: Calculate new launch angle
        double launchAngleNew = Math.atan(vy / vxNew);
        launchAngleNew = clamp(launchAngleNew, minClampAngle, maxClampAngle);

        // Step 8: Calculate new distance traveled
        double xNew = vxNew * timeToGoal;

        // Step 9: Calculate new launch speed with clamped angle
        double cosAlphaNew = Math.cos(launchAngleNew);
        double tanAlphaNew = Math.tan(launchAngleNew);
        numerator = GRAVITY * xNew * xNew;
        denominator = 2.0 * cosAlphaNew * cosAlphaNew *
                (xNew * tanAlphaNew - goalHeight);
        double launchSpeedNew = Math.sqrt(numerator / denominator);

        // Step 10: Convert to hardware commands
        ShootingParameters result = new ShootingParameters();
        result.launchAngle = launchAngleNew;
        result.launchSpeed = launchSpeedNew;
        result.hoodServoPosition = calculateHoodServoPosition(launchAngleNew);
        result.flywheelSpeed = calculateFlywheelSpeed(launchSpeedNew);
        result.distanceToGoal = distanceToGoal;

        return result;
    }

    // ========================================================================
    // HARDWARE CONVERSION METHODS
    // ========================================================================

    /**
     * Calculate hood servo position for a given launch angle.
     *
     * @param angle Launch angle in radians
     * @return Servo position (0.0 to 1.0)
     */
    private double calculateHoodServoPosition(double angle) {
        double angleDegrees = Math.toDegrees(angle);

        // Linear interpolation between calibration points
        double slope = (minServoPosition - maxServoPosition) / (minHoodAngle - maxHoodAngle);
        double position = slope * (angleDegrees - minHoodAngle) + minServoPosition;

        // Clamp to valid servo range
        return clamp(position, 0.0, 1.0);
    }

    /**
     * Calculate flywheel speed for a given launch speed.
     *
     * @param launchSpeed Ball launch speed (in/s)
     * @return Flywheel speed in RPM (or whatever you calibrated)
     */
    private double calculateFlywheelSpeed(double launchSpeed) {
        // Linear relationship: flywheelSpeed = a * launchSpeed + b
        return flywheelSpeedSlope * launchSpeed + flywheelSpeedIntercept;
    }

    // ========================================================================
    // UTILITY METHODS
    // ========================================================================

    /**
     * Clamp a value between min and max.
     */
    private double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    /**
     * Normalize an angle to [-PI, PI]
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}