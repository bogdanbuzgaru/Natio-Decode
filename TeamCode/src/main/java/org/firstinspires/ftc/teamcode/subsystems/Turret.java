package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double difPos;
    private double targetAngle;
    private double lastAngle;
    private double angle;
    private double heading;
    private double offsetAngle;

    private final double ratio = 165.5;

    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");

        turretServo1.setPosition(0.5);
        turretServo2.setPosition(0.5);
        turretServo3.setPosition(0.5);
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    private double error(){
        double error = heading - angle + offsetAngle;
        if(error > 180){
            error -= 360;
        } else if (error < -180){
            error += 360;
        }
        return error;
    }
    public boolean turnLeft(){
        return error() < 0;
    }
    public boolean turnRight(){
        return error() > 0;
    }
    public void update(){
        double theAngle = targetAngle + offsetAngle;
        if(turnLeft()){
            double position = 0.5 + 0.5 * Math.abs(targetAngle / ratio);
            turretServo1.setPosition(position);
            turretServo2.setPosition(position) ;
            turretServo3.setPosition(position);
        } else if (turnRight()){
            double position = 0.5 - 0.5 * Math.abs(targetAngle / ratio);
            turretServo1.setPosition(position);
            turretServo2.setPosition(position) ;
            turretServo3.setPosition(position);
        }
    }
    public void goNeutral(){
        turretServo1.setPosition(0.5);
        turretServo2.setPosition(0.5);
        turretServo3.setPosition(0.5);
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
    public void setTargetAngle(double angle) {
        lastAngle = this.targetAngle;
        if (angle > 180)
            angle -= 360;
        if (angle < -180)
            angle += 360;
        if(Math.signum(lastAngle) != Math.signum(angle) && 180 - Math.abs(angle) < 8)
            this.targetAngle = lastAngle + 180 - Math.abs(angle);
        else
            this.targetAngle = angle;    }
    public void setAngle(double angle) {
        this.angle = angle;
    }
    public void setAuto(){
        turretServo1.setPosition(0.5 + 0.11765);
        turretServo2.setPosition(0.5 + 0.11765);
        turretServo3.setPosition(0.5 + 0.11765);
    }
    public void setAutoBlue(){
        turretServo1.setPosition(0.5 - 0.11765);
        turretServo2.setPosition(0.5 - 0.11765);
        turretServo3.setPosition(0.5 - 0.11765);
    }
    public void setDifPos(double difPos) {
        this.difPos = difPos;
    }
}
