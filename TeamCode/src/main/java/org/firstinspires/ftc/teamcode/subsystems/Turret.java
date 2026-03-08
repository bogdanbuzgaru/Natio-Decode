package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double difPos;
    private double targetAngle;
    private double heading;
    private double offsetAngle;

    // Full servo travel (0→1) spans this many degrees of turret rotation.
    // Derived from hardware gearing: 270° physical × (gear ratio) ≈ 192.86°.
    private final double ratio = 192.8571428571429;

    // Servo offset for pre-defined auto positions (≈ 22.7°, or ~11.8% of travel).
    private static final double AUTO_OFFSET = 0.11765;

    // Smoothing: 0 = no change, 1 = instant jump.  Tune between 0.08-0.25.
    private static final double SMOOTHING_ALPHA = 0.15;

    // Dead-zone in degrees — ignore errors smaller than this to prevent jitter.
    private static final double DEADZONE_DEG = 1.0;

    private double smoothedPosition = 0.5;

    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");

        turretServo1.setPosition(0.5);
        turretServo2.setPosition(0.5);
        turretServo3.setPosition(0.5);
        smoothedPosition = 0.5;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    /**
     * Wraps an angle into the [-180, 180] range.
     */
    private static double wrapAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Sets the target angle (signed, already wrapped by Position.getTargetAngle()).
     * Applies proper [-180,180] wrapping for safety.
     */
    public void setTargetAngle(double angle) {
        this.targetAngle = wrapAngle(angle);
    }

    /**
     * Core update — call once per loop.
     *
     * Computes the desired servo position from the signed targetAngle,
     * applies a dead-zone to avoid jitter, and smooths the movement with
     * an exponential low-pass filter.
     */
    public void update(){
        double totalAngle = wrapAngle(targetAngle + offsetAngle);

        // Dead-zone: if the correction is tiny, don't move
        if (Math.abs(totalAngle) < DEADZONE_DEG) {
            // Hold current smoothed position — no update
            applyPosition(smoothedPosition);
            return;
        }

        // Map angle to servo position.
        // Positive angle → position < 0.5 (turn one way)
        // Negative angle → position > 0.5 (turn the other way)
        double rawPosition = 0.5 - (totalAngle / ratio);

        // Clamp to valid servo range [0, 1]
        rawPosition = Math.max(0.0, Math.min(1.0, rawPosition));

        // Exponential low-pass filter for smooth motion
        smoothedPosition = SMOOTHING_ALPHA * rawPosition + (1.0 - SMOOTHING_ALPHA) * smoothedPosition;

        applyPosition(smoothedPosition);
    }

    /**
     * Sends the position to all three turret servos.
     */
    private void applyPosition(double position) {
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }

    public void goNeutral(){
        smoothedPosition = 0.5;
        applyPosition(0.5);
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getAngleRatio() {
        return targetAngle / ratio;
    }

    public double getPosition(){
        return turretServo1.getPosition();
    }

    public void setOffsetAngle(double offsetAngle) {
        this.offsetAngle = offsetAngle;
    }

    /** @deprecated Use {@link #setTargetAngle(double)} with a signed angle from
     *  {@code Position.getTargetAngle()} instead. */
    @Deprecated
    public void setAngle(double angle) {
        // Kept for API compatibility (used by callers).
        // No longer used internally — targetAngle is now the signed error.
    }

    public void setAuto(){
        smoothedPosition = 0.5 + AUTO_OFFSET;
        applyPosition(smoothedPosition);
    }

    public void setAutoBlue(){
        smoothedPosition = 0.5 - AUTO_OFFSET;
        applyPosition(smoothedPosition);
    }

    public void setDifPos(double difPos) {
        this.difPos = difPos;
    }
}
