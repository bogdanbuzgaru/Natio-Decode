package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private DcMotorEx flywheelMotor1, flywheelMotor2;
    private Servo barrier, hood;
    private double ticks;
    private double hoodPosition;
    public Shooter (HardwareMap hardwareMap){
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        barrier = hardwareMap.get(Servo.class, "barrier");
        hood = hardwareMap.get(Servo.class, "hood");

        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheelMotor1.setDirection(DcMotorEx.Direction.REVERSE);

        lowerBarrier();
    }
    public void update(){
        flywheelMotor1.setVelocity(ticks);
        flywheelMotor2.setVelocity(ticks);
        hood.setPosition(hoodPosition);
    }
    public void setTicks(double ticks) {
        this.ticks = ticks;
    }
    public void lowerBarrier(){
        barrier.setPosition(0.3);
    }
    public void raiseBarrier(){
        barrier.setPosition(0.7);
    }
    public void raiseHood(){
        hood.setPosition(1);
    }
    public void lowerHood(){
        hood.setPosition(0.29);
    }
    public void setHoodPosition(double hoodPosition) {
        this.hoodPosition = hoodPosition;
    }
}
