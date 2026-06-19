package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pid.PIDController;
import org.firstinspires.ftc.teamcode.pid.SimpleMotorFeedforward;

public class Shooter {
    private DcMotorEx flywheelMotor1, flywheelMotor2;
    private Servo barrier, hood;
    private double ticks = 1000;
    private double hoodPosition = 0;
    private double targetHood;
    private PIDController p = new PIDController(kp, 0, 0);
    private double voltagee;
    private double pCoef = 120;
    public static  double ks = 0.226, kv = 0.0021039446333333, ka = 0.005, kp = 0.05, velocity, nominalVoltage = 11.2;
    public Shooter (HardwareMap hardwareMap){
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        barrier = hardwareMap.get(Servo.class, "barrier");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);
//
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pCoef,0,0,13.3));
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pCoef,0,0,13.3));


        lowerBarrier();
    }
    public void update(){
//        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks, kv, ka);
//        p.setPID(kp, 0, 0);
//
//
//        double vecocity = flywheelMotor1.getVelocity();
//
//        double p_output = p.calculate(vecocity, ticks);
//        double ff_ouput = ff.calculate(ticks);
//
//        flywheelMotor2.setPower((p_output + ff_ouput) * (nominalVoltage / voltagee));
//        flywheelMotor1.setPower((p_output + ff_ouput) * (nominalVoltage / voltagee));
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pCoef,0,0,13.3));
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pCoef,0,0,13.3));

        flywheelMotor1.setVelocity(ticks);
        flywheelMotor2.setVelocity(ticks);
        adaptiveHood();
    }
    public void changeCoef(boolean far){
        if(far){
            pCoef = 170;
        }else{
            pCoef = 120;
        }
    }

    public void setVoltagee(double voltagee) {
        this.voltagee = voltagee;
    }

    public void updateMotor(){
        flywheelMotor1.setVelocity(ticks);
        flywheelMotor2.setVelocity(ticks);
        setAutoFarHood();
    }
    public void setTicks(double ticks) {
        if(ticks < 1500){
            ticks+= 50;
        }else{
            ticks+=25;
        }
        if(Math.abs(this.ticks - ticks) >= 30){
            this.ticks = ticks;
        }
    }

    public double getTicks() {
        return flywheelMotor1.getVelocity();
    }
    public double getHoodPosition(){
        return hood.getPosition();
    }
    public double getTarget(){
        return targetHood;
    }
    private void adaptiveHood(){
        double error = Math.abs((flywheelMotor1.getVelocity() - ticks) * 0.0005);
        if(flywheelMotor1.getVelocity()  <= (1150) * 1.45){
            targetHood = 0;
        }else {
            targetHood = 0.8 * ((ticks - (1150 * 1.45)) / (550*1.45)) + 0.2;
        }
        targetHood = Math.min(targetHood, 1);
        hood.setPosition(targetHood);
    }
    public void lowerBarrier(){
        barrier.setPosition(0.53);
    }
    public void raiseBarrier(){
        barrier.setPosition(0.358);
    }
    public void middleBar(){
        barrier.setPosition(0.5);
        hood.setPosition(0.5);
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
    public void setAutoFarHood(){
        hood.setPosition(0.94);
    }
    public void lowerHood(){
            ticks = 0;
            hood.setPosition(0);
    }
    public void setHoodPosition(Gamepad gamepad) {
        if(gamepad.dpadUpWasPressed()) {
            hoodPosition += 0.1;
            hood.setPosition(hoodPosition);
        }else if (gamepad.dpadDownWasPressed()){
            hoodPosition -= 0.1;
            hood.setPosition(hoodPosition);
        }
        if (gamepad.leftBumperWasPressed()){
            ticks -=50;
        }else if (gamepad.rightBumperWasPressed()){
            ticks +=50;
        }

        if(gamepad.crossWasPressed()){
            raiseBarrier();
        }else if (gamepad.circleWasPressed()){
            lowerBarrier();
        }
        flywheelMotor1.setVelocity(ticks);
        flywheelMotor2.setVelocity(ticks);
    }
}
