package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double targetAngle;      // where we WANT to be
    private double offsetAngle;

    // --- Motion profile state ---
    private double currentProfiledAngle = 0.0;   // where the servo is currently being driven to
    private double currentVelocity = 0.0;        // deg/s, always >= 0 (direction handled separately)
    private double lastWrittenPosition = -1.0;   // for write-caching
    private final ElapsedTime profileTimer = new ElapsedTime();

    // --- Tuning knobs: start conservative, increase until it's fast but not jerky ---
    private static final double MAX_VELOCITY = 345.0;   // deg/s
    private static final double ACCELERATION = 400.0;   // deg/s^2
    private static final double POSITION_DEADBAND = 0.3; // degrees, prevents micro-jitter at target
    private static final double WRITE_THRESHOLD = 0.0015; // servo units, skip redundant writes
    private static final double MAX_DT = 0.1;            // clamp dt after any stall/gap

    private final double HALF_RANGE_DEGREES = 190.1785714285714;
    private final double SERVO_CENTER = 0.500000;
    private boolean isRed;
    private double heading;
    private boolean blue = false;
    private boolean addAngle = false;

    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");
        profileTimer.reset();
//        goNeutral();
    }

    public void setBlue(boolean blue){
        isRed = !blue;
    }

    public void setTargetAngle(double angleRobotRelative) {
        this.targetAngle = normalizeAngle(angleRobotRelative);
    }

    public void setHeading(double heading, boolean red) {
        this.heading = heading;
        isRed = red;
    }

    public void setOffsetAngle(double offsetAngle) { this.offsetAngle = offsetAngle; }

    public void addForOffset(Gamepad gamepad){
        if (gamepad.leftBumperWasPressed()) addAngle = !addAngle;
    }

    public boolean isAddAngle() { return addAngle; }

    private double getFinalAngle() {
        double finalAngle = targetAngle + offsetAngle;
        if (isRed && heading < 70 && heading > -90) {
            finalAngle = finalAngle;
        }
        if (!isRed && (heading > 110 || heading < -90)) {
            finalAngle += 3;
        }
        return normalizeAngle(finalAngle);
    }

    // angleToPos() and goNeutral() now just RETARGET instead of snapping directly.
    // update() is the only method that ever writes to the servos.
    public void angleToPos(double angle){
        setTargetAngle(angle);
    }

    public void goNeutral(){
        turretServo1.setPosition(0.5);
        turretServo2.setPosition(0.5);
        turretServo3.setPosition(0.5);
    }

    public void update(){
        double dt = profileTimer.seconds();
        profileTimer.reset();
        dt = Math.min(dt, MAX_DT); // guard against a huge dt if update() wasn't called for a while

        double desiredAngle = getFinalAngle();
        double distanceToTarget = desiredAngle - currentProfiledAngle;
        double absDistance = Math.abs(distanceToTarget);

        double brakingDistance = (currentVelocity * currentVelocity) / (2.0 * ACCELERATION);

        if (absDistance > POSITION_DEADBAND) {
            if (absDistance <= brakingDistance) {
                // Decelerate phase
                currentVelocity -= ACCELERATION * dt;
                currentVelocity = Math.max(currentVelocity, 0.0);
            } else {
                // Accelerate / cruise phase
                currentVelocity += ACCELERATION * dt;
                currentVelocity = Math.min(currentVelocity, MAX_VELOCITY);
            }

            double step = currentVelocity * dt;
            if (distanceToTarget > 0) {
                currentProfiledAngle = Math.min(currentProfiledAngle + step, desiredAngle);
            } else {
                currentProfiledAngle = Math.max(currentProfiledAngle - step, desiredAngle);
            }
        } else {
            currentVelocity = 0.0;
            currentProfiledAngle = desiredAngle; // snap the last tiny bit, avoids infinite creep
        }

        double servoOffset = (currentProfiledAngle / HALF_RANGE_DEGREES) * 0.5000;
        double servoPosition = SERVO_CENTER - servoOffset;
        servoPosition = Math.max(0.00000, Math.min(1.00000, servoPosition));

        if (Math.abs(servoPosition - lastWrittenPosition) > WRITE_THRESHOLD) {
            turretServo1.setPosition(servoPosition);
            turretServo2.setPosition(servoPosition);
            turretServo3.setPosition(servoPosition);
            lastWrittenPosition = servoPosition;
        }
    }

    private double normalizeAngle(double degrees) {
        while (degrees > 165) degrees -= 330;
        while (degrees < -165) degrees += 330;
        return degrees;
    }

    public double getTargetAngle() { return targetAngle; }
    public double getFinalAngleDegrees() { return getFinalAngle(); }
    public double getCurrentProfiledAngle() { return currentProfiledAngle; }

    public double getPosition1(){ return turretServo1.getPosition(); }
    public double getPosition2(){ return turretServo2.getPosition(); }
    public double getPosition3(){ return turretServo3.getPosition(); }

    public void setAuto(){ setTargetAngle(SERVO_CENTER + 0.1368421052631579 == 0 ? 0 : angleFromServoOffset(0.1368421052631579)); }
    public void setAutoBlue(){ setTargetAngle(angleFromServoOffset(-0.1368421052631579)); }
    public void setFarAuto(){ setTargetAngle(angleFromServoOffset(0.2143157894736842)); }
    public void secondAutoFar(){ setTargetAngle(angleFromServoOffset(0.2083157894736842)); }
    public void setFarAutoBlue(){ setTargetAngle(angleFromServoOffset(-0.2143157894736842)); }
    public void secondAutoFarBlue(){ setTargetAngle(angleFromServoOffset(-0.2083157894736842)); }

    // Converts your old raw servo-offset magic numbers back into degrees for setTargetAngle().
    private double angleFromServoOffset(double servoOffset) {
        return -servoOffset * HALF_RANGE_DEGREES / 0.5;
    }
}