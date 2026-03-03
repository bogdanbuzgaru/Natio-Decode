package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private DcMotorEx flywheelMotor1, flywheelMotor2;
    private Servo barrier, hood;
    private double ticks;
    private double hoodPosition;
    private double targetHood;
    public Shooter (HardwareMap hardwareMap){
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        barrier = hardwareMap.get(Servo.class, "barrier");
        hood = hardwareMap.get(Servo.class, "hood");

        flywheelMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        lowerBarrier();
    }
    public void update(){
        flywheelMotor1.setVelocity(ticks);
        flywheelMotor2.setVelocity(ticks);
        adaptiveHood();
        hood.setPosition(targetHood);
    }
    public void setTicks(double ticks) {
        if(Math.abs(this.ticks - ticks) >= 50){
            this.ticks = ticks;
        }
    }

    public double getTicks() {
        return ticks;
    }
    private void adaptiveHood(){
        double error = flywheelMotor1.getVelocity() - ticks;
        targetHood = Math.min(ticks/2400 + Math.min(hoodPosition + error * 0.0005, 1), 1);
    }
    public void lowerBarrier(){
        barrier.setPosition(0.6);
    }
    public void raiseBarrier(){
        barrier.setPosition(0.4);
    }
    public void setTicks(Gamepad gamepad){
        if(gamepad.dpadLeftWasPressed()){
            ticks = 1456;
            flywheelMotor1.setVelocity(ticks);
            flywheelMotor2.setVelocity(ticks);
        }else if(gamepad.circleWasPressed()){
            ticks = 1619;
            flywheelMotor1.setVelocity(ticks);
            flywheelMotor2.setVelocity(ticks);
        }else if(gamepad.dpadRightWasPressed()){
            ticks = 2300;
            flywheelMotor1.setVelocity(ticks);
            flywheelMotor2.setVelocity(ticks);
        }else if(gamepad.crossWasPressed()){
            ticks = 1280;
            flywheelMotor1.setVelocity(ticks);
            flywheelMotor2.setVelocity(ticks);
        }
    }
    public void setHoodPosition(double hoodPosition) {
//        this.hoodPosition = hoodPosition;
    }
}
