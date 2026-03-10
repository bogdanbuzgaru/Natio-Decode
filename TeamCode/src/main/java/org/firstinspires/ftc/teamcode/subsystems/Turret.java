package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double targetAngleRobotRelative;  // Target angle relative to robot
    private double offsetAngle;                // Velocity compensation offset

    // CALIBRATION: Full servo range (0.0 to 1.0) = 385.71 degrees
    // Half range (0.5 to 1.0 or 0.0 to 0.5) = 192.86 degrees
    private final double HALF_RANGE_DEGREES = 192.8571428571429;
    private final double SERVO_CENTER = 0.5;

    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");

        goNeutral();
    }

    /**
     * Set the target angle (robot-relative, already has heading subtracted)
     * This comes from Position.getTargetAngle()
     */
    public void setTargetAngle(double angleRobotRelative) {
        this.targetAngleRobotRelative = normalizeAngle(angleRobotRelative);
    }

    /**
     * Set velocity compensation offset
     * This comes from Position.getOffetAngle()
     */
    public void setOffsetAngle(double offsetAngle) {
        this.offsetAngle = offsetAngle;
    }

    /**
     * Calculate final angle including velocity compensation
     */
    private double getFinalAngle() {
        double finalAngle = targetAngleRobotRelative + offsetAngle;
        return normalizeAngle(finalAngle);
    }

    /**
     * Update servo positions to point at target angle
     */
    public void update(){
        double finalAngle = getFinalAngle();

        // Convert angle to servo position
        // Formula: servoPosition = 0.5 + (angle / halfRangeDegrees) * 0.5
        //
        // Examples:
        // angle = 0°     → servo = 0.5 + (0 / 192.86) * 0.5 = 0.5 (center)
        // angle = +96.43° → servo = 0.5 + (96.43 / 192.86) * 0.5 = 0.75 (quarter right)
        // angle = -96.43° → servo = 0.5 + (-96.43 / 192.86) * 0.5 = 0.25 (quarter left)
        // angle = +192.86° → servo = 0.5 + (192.86 / 192.86) * 0.5 = 1.0 (full right)
        // angle = -192.86° → servo = 0.5 + (-192.86 / 192.86) * 0.5 = 0.0 (full left)

        double servoOffset = (finalAngle / HALF_RANGE_DEGREES) * 0.5;

        // IMPORTANT: Check if your turret mechanics require negative sign
        // If positive angle should decrease servo (turn left), use minus:
        // double servoPosition = SERVO_CENTER - servoOffset;
        // If positive angle should increase servo (turn right), use plus:
        double servoPosition = SERVO_CENTER + servoOffset;

        // Clamp to valid servo range
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        // Set all three servos
        turretServo1.setPosition(servoPosition);
        turretServo2.setPosition(servoPosition);
        turretServo3.setPosition(servoPosition);
    }

    /**
     * Return turret to center position
     */
    public void goNeutral(){
        turretServo1.setPosition(SERVO_CENTER);
        turretServo2.setPosition(SERVO_CENTER);
        turretServo3.setPosition(SERVO_CENTER);
    }

    /**
     * Normalize angle to [-180, 180]
     */
    private double normalizeAngle(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    /**
     * Get current target angle (for telemetry)
     */
    public double getTargetAngle() {
        return targetAngleRobotRelative;
    }

    /**
     * Get final angle including offset (for telemetry)
     */
    public double getFinalAngleDegrees() {
        return getFinalAngle();
    }

    /**
     * Get servo positions (for telemetry)
     */
    public double getPosition1(){ return turretServo1.getPosition(); }
    public double getPosition2(){ return turretServo2.getPosition(); }
    public double getPosition3(){ return turretServo3.getPosition(); }

    /**
     * Preset positions for auto
     */
    public void setAuto(){
        double position = SERVO_CENTER + 0.11765;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }

    public void setAutoBlue(){
        // -22.69° offset
        double position = SERVO_CENTER - 0.11765;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
}