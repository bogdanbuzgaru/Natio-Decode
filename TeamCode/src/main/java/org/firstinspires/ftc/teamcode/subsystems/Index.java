package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Index {
    private DcMotorEx indexMotor;
    private boolean lower = false;
    private Servo indexMove;
    public Index (HardwareMap hardwareMap){
        indexMotor = hardwareMap.get(DcMotorEx.class, "index");
        indexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexMove = hardwareMap.get(Servo.class, "indexMove");
        indexMove.setPosition(0.47);
    }

    public void setLower(boolean lower) {
        this.lower = lower;
    }
    public void normalIndex(){
        indexMove.setPosition(0.52);
    }
    public void lowerIndex(){
        indexMove.setPosition(0.2);
    }
    public void feed(Gamepad gamepad){
        if (!lower) {
            if (gamepad.right_trigger > 0.01)
                indexMotor.setPower(0.8);
            else if (gamepad.left_trigger > 0.01)
                indexMotor.setPower(-gamepad.left_trigger);
            else
                stop();
        }else{
            if (gamepad.right_trigger > 0.01)
                indexMotor.setPower(0.0);
            else if (gamepad.left_trigger > 0.01)
                indexMotor.setPower(-gamepad.left_trigger);
            else
                stop();
        }
    }
    public void autoFeed(){
        indexMotor.setPower(1);
    }
    public void stop(){
        indexMotor.setPower(0);
    }
    public void slowFeed(){
        indexMotor.setPower(0.5);
    }
}
//Davidescu