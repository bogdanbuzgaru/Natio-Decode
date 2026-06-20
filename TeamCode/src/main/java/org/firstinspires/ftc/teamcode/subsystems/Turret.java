package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double targetAngle;  // Target angle relative to robot
    private double offsetAngle;                // Velocity compensation offset

    private final double HALF_RANGE_DEGREES = 173.1785714285714;       //TODO  173.1785714285714
    private final double SERVO_CENTER = 0.500000;
    private boolean isRed;
    private double heading;
    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");

        goNeutral();
    }

    public void setTargetAngle(double angleRobotRelative) {
        this.targetAngle = normalizeAngle(angleRobotRelative);
    }

    public void setHeading(double heading, boolean red) {
        this.heading = heading;
        isRed = red;
    }

    public void setOffsetAngle(double offsetAngle) {
        this.offsetAngle = offsetAngle;
    }

    private double getFinalAngle() {
        double finalAngle = targetAngle + offsetAngle;
        if(Math.abs(targetAngle) >= 30 && Math.signum(targetAngle) == -1){
            targetAngle += (targetAngle - 30) * 0.125;
        }else if(Math.abs(targetAngle) >= 30 && Math.signum(targetAngle) == 1){
            targetAngle -= (targetAngle - 30) * 0.125;
        }
        if(isRed && Math.abs(heading) < 90){
            finalAngle -= 10;
        }else if (!isRed && Math.abs(heading) > 90){
            finalAngle -= 10;
        }

        return normalizeAngle(finalAngle);
    }

    public void update(){
        double finalAngle = getFinalAngle();

        double servoOffset = (finalAngle / HALF_RANGE_DEGREES) * 0.5000;
        double servoPosition = SERVO_CENTER - servoOffset;

        servoPosition = Math.max(0.00000, Math.min(1.00000, servoPosition));

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
        return targetAngle;
    }

    public double getFinalAngleDegrees() {
        return getFinalAngle();
    }

    public double getPosition1(){ return turretServo1.getPosition(); }
    public double getPosition2(){ return turretServo2.getPosition(); }
    public double getPosition3(){ return turretServo3.getPosition(); }


    public void setAuto(){
        double position = SERVO_CENTER + 0.1368421052631579;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }

    public void setAutoBlue(){
        double position = SERVO_CENTER - 0.1368421052631579;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
    public void setFarAuto(){
        double position = SERVO_CENTER + 0.2143157894736842;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
    public void secondAutoFar(){
        double position = SERVO_CENTER + 0.2083157894736842;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
    public void setFarAutoBlue(){
        double position = SERVO_CENTER - 0.2143157894736842;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
    public void secondAutoFarBlue(){
        double position = SERVO_CENTER - 0.2083157894736842;
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
}