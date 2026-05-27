package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Index {
    private DcMotorEx indexMotor;
    public Index (HardwareMap hardwareMap){
        indexMotor = hardwareMap.get(DcMotorEx.class, "index");
        indexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void feed(Gamepad gamepad){
        if(gamepad.right_trigger > 0.01)
            indexMotor.setPower(0.2);
        else if (gamepad.left_trigger > 0.01)
            indexMotor.setPower(-gamepad.left_trigger);
        else
            stop();
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
