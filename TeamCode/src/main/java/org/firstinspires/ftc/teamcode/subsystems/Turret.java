package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math.ShooterCalculations;

public class Turret {
    private Servo turretServo1, turretServo2, turretServo3;
    private double difPos;
    private double angle;
    private double lastAngle;

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
    public boolean turnLeft(){
        return heading < angle;
    }
    public boolean turnRight(){
        return heading > angle || heading < angle - 180;
    }
    public void update(){
        if(turnLeft()){
            turretServo1.setPosition(0.5 + 0.5 * Math.abs(angle / ratio));
            turretServo2.setPosition(0.5 + 0.5 * Math.abs(angle / ratio)) ;     //TODO implement Last angle to not trigger bugs at 180 and go further
            turretServo3.setPosition(0.5 + 0.5 * Math.abs(angle / ratio));
        } else if (turnRight()){
            turretServo1.setPosition(0.5 - 0.5 * Math.abs(angle / ratio));
            turretServo2.setPosition(0.5 - 0.5 * Math.abs(angle / ratio));
            turretServo3.setPosition(0.5 - 0.5 * Math.abs(angle / ratio));
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

    public double getAngle() {
        return angle;
    }
    public double getAngleRatio() {
        return angle / ratio;
    }

    public void setAngle(double angle) {
        lastAngle = angle;
        if(angle > 180){
            angle = 360 - angle;
        }
        this.angle = angle;
    }
    public void setDifPos(double difPos) {
        this.difPos = difPos;
    }
}
