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
    private double angle, lastAngle;

    public Turret(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");


        turretServo1.setPosition(0.5);
        turretServo2.setPosition(0.5);
        turretServo3.setPosition(0.5);
    }

    public void update(){
        if(angle > 0){
            turretServo1.setPosition(0.5 - angle / 197.25);
            turretServo2.setPosition(0.5 - angle / 197.25);     //TODO implement Last angle to not trigger bugs at 180 and go further
            turretServo3.setPosition(0.5 - angle / 197.25);
        } else{
            turretServo1.setPosition(0.5 + angle / 197.25);
            turretServo2.setPosition(0.5 + angle / 197.25);
            turretServo3.setPosition(0.5 + angle / 197.25);
        }
    }
    public void setAngle(double angle) {
        lastAngle = angle;
        this.angle = angle;
    }
    public void setDifPos(double difPos) {
        this.difPos = difPos;
    }
}
