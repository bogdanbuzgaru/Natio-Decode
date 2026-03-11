package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double targetAngleRobotRelative;  // Target angle relative to robot
    private double offsetAngle;                // Velocity compensation offset

    private final double HALF_RANGE_DEGREES = 190.1785714285714;       //TODO Check for lower value
    private final double SERVO_CENTER = 0.5;

    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");

        goNeutral();
    }

    public void setTargetAngle(double angleRobotRelative) {
        this.targetAngleRobotRelative = normalizeAngle(angleRobotRelative);
    }


    public void setOffsetAngle(double offsetAngle) {
        this.offsetAngle = offsetAngle;
    }


    private double getFinalAngle() {
        double finalAngle = targetAngleRobotRelative + offsetAngle;
        return normalizeAngle(finalAngle);
    }

    public void update(){
        double finalAngle = getFinalAngle();
        double servoOffset = (finalAngle / HALF_RANGE_DEGREES) * 0.5;
        double servoPosition = SERVO_CENTER + servoOffset;

        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        turretServo1.setPosition(servoPosition);
        turretServo2.setPosition(servoPosition);
        turretServo3.setPosition(servoPosition);
    }

    public void goNeutral(){
        turretServo1.setPosition(SERVO_CENTER);
        turretServo2.setPosition(SERVO_CENTER);
        turretServo3.setPosition(SERVO_CENTER);
    }

    private double normalizeAngle(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    public double getTargetAngle() {
        return targetAngleRobotRelative;
    }

    public double getFinalAngleDegrees() {
        return getFinalAngle();
    }

    public double getPosition1(){ return turretServo1.getPosition(); }
    public double getPosition2(){ return turretServo2.getPosition(); }
    public double getPosition3(){ return turretServo3.getPosition(); }

    public void setAuto(){
        double position = SERVO_CENTER + 0.11765;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }

    public void setAutoBlue(){
        double position = SERVO_CENTER - 0.11765;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
}