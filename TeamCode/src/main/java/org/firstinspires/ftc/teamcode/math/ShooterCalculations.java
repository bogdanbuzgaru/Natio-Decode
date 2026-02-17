package org.firstinspires.ftc.teamcode.math;

public class ShooterCalculations {

    private static final double GRAVITY = 386.1; // in/s^2 (9.8 m/s^2 converted)
    private double minHoodAngle;  // degrees
    private double maxHoodAngle;  // degrees
    private double minServoPosition; // servo position at min angle
    private double maxServoPosition; // servo position at max angle
    private double flywheelSpeedSlope;     // coefficient 'a'
    private double flywheelSpeedIntercept;  // coefficient 'b'
    private double launchSpeedNew;

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

    public static class ShootingParameters {
        public double launchAngle;         // radians
        public double launchSpeed;         // in/s
        public double hoodServoPosition;   // servo position for hood
        public double flywheelSpeed;       // speed for both flywheel motors
        public double distanceToGoal;      // calculated distance (for telemetry)
        public double angleToGoal;         // Basic angle to goal (no velocity compensation)
        public double turretOffsetAngle;   // Velocity compensation offset to ADD
        public double targetTurretAngle;   // Final angle to aim at = angleToGoal + turretOffsetAngle

        public double getFlywheelSpeed() {
            return flywheelSpeed;
        }

        public double getLaunchAngleDegrees() {
            return Math.toDegrees(launchAngle);
        }

        public double getAngleToGoalDegrees() {
            return Math.toDegrees(angleToGoal);
        }

        public double getTurretOffsetDegrees() {
            return Math.toDegrees(turretOffsetAngle);
        }

        public double getTargetTurretDegrees() {
            return Math.toDegrees(targetTurretAngle);
        }
    }

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

        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distanceToGoal = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.atan2(dy, dx);

        double alpha = Math.atan((2.0 * goalHeight / distanceToGoal) - Math.tan(goalEntryAngle));

        double cosAlpha = Math.cos(alpha);
        double tanAlpha = Math.tan(alpha);
        double numerator = GRAVITY * distanceToGoal * distanceToGoal;
        double denominator = 2.0 * cosAlpha * cosAlpha * (distanceToGoal * tanAlpha - goalHeight);
        double v0 = Math.sqrt(numerator / denominator);

        double robotSpeed = Math.sqrt(robotVelocityX * robotVelocityX +
                robotVelocityY * robotVelocityY);
        double robotVelocityAngle = Math.atan2(robotVelocityY, robotVelocityX);

        double deltaAngle = robotVelocityAngle - angleToGoal;
        double vRadial = -Math.cos(deltaAngle) * robotSpeed;
        double vTangential = Math.sin(deltaAngle) * robotSpeed;

        double vx = v0 * Math.cos(alpha);
        double timeToGoal = distanceToGoal / vx;

        double vxCompensatedRadial = vx + vRadial;
        double vxNew = Math.sqrt(vxCompensatedRadial * vxCompensatedRadial +
                vTangential * vTangential);

        double vy = v0 * Math.sin(alpha);

        double launchAngleNew = Math.atan(vy / vxNew);
        launchAngleNew = clamp(launchAngleNew, minClampAngle, maxClampAngle);

        double xNew = vxNew * timeToGoal;

        double cosAlphaNew = Math.cos(launchAngleNew);
        double tanAlphaNew = Math.tan(launchAngleNew);
        numerator = GRAVITY * xNew * xNew;
        denominator = 2.0 * cosAlphaNew * cosAlphaNew *
                (xNew * tanAlphaNew - goalHeight);
        launchSpeedNew = Math.sqrt(numerator / denominator);

        double turretOffsetAngle = Math.atan(vTangential / vxCompensatedRadial);

        ShootingParameters result = new ShootingParameters();
        result.launchAngle = launchAngleNew;
        result.launchSpeed = launchSpeedNew;
        result.hoodServoPosition = calculateHoodServoPosition(launchAngleNew);
        result.flywheelSpeed = calculateFlywheelSpeed();
        result.distanceToGoal = distanceToGoal;


        result.angleToGoal = angleToGoal;
        result.turretOffsetAngle = turretOffsetAngle;
        result.targetTurretAngle = angleToGoal + turretOffsetAngle;

        return result;
    }

    private double calculateHoodServoPosition(double angle) {
        double angleDegrees = Math.toDegrees(angle);

        double slope = (minServoPosition - maxServoPosition) / (minHoodAngle - maxHoodAngle);
        double position = slope * (angleDegrees - minHoodAngle) + minServoPosition;

        return clamp(position, 0.0, 1.0);
    }

    private double calculateFlywheelSpeed(){
        // Linear relationship: flywheelSpeed = a * launchSpeed + b
        return flywheelSpeedSlope * launchSpeedNew + flywheelSpeedIntercept;
    }
    private double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}