package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double difPos;
    private double targetAngle;
    private double lastAngle;
    private double angle;
    private double heading;
    private final double ratio = 190.1785714285714;

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
        double error = heading - angle;
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
        if(turnLeft()){
            turretServo1.setPosition(0.5 + 0.5 * Math.abs(targetAngle / ratio));
            turretServo2.setPosition(0.5 + 0.5 * Math.abs(targetAngle / ratio)) ;     //TODO implement Last angle to not trigger bugs at 180 and go further
            turretServo3.setPosition(0.5 + 0.5 * Math.abs(targetAngle / ratio));
        } else if (turnRight()){
            turretServo1.setPosition(0.5 - 0.5 * Math.abs(targetAngle / ratio));
            turretServo2.setPosition(0.5 - 0.5 * Math.abs(targetAngle / ratio));
            turretServo3.setPosition(0.5 - 0.5 * Math.abs(targetAngle / ratio));
        }
    }
    public void goLeft(){
        turretServo1.setPosition(0.3);
        turretServo2.setPosition(0.3);
        turretServo3.setPosition(0.3);
    }
    public void goRight(){
        turretServo1.setPosition(0.7);
        turretServo2.setPosition(0.7);
        turretServo3.setPosition(0.7);
    }

    public double getTargetAngle() {
        return targetAngle;
    }
    public double getAngleRatio() {
        return targetAngle / ratio;
    }
    public void setTargetAngle(double angle) {
        lastAngle = this.targetAngle;
        if (angle > 180)
            angle -= 360;
        if (angle < -180)
            angle += 360;
        this.targetAngle = angle;
    }
    public void setAngle(double angle) {
        this.angle = angle;
    }
    public void setDifPos(double difPos) {
        this.difPos = difPos;
    }
}
