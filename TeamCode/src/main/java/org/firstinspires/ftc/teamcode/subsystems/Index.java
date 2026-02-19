package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Index {
    private DcMotorEx indexMotor;
    public Index (HardwareMap hardwareMap){
        indexMotor = hardwareMap.get(DcMotorEx.class, "index");
        indexMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void feed(double val){
        if(val > 0.1)
            indexMotor.setPower(val);
        else
            stop();
    }
    public void eject(){
        indexMotor.setPower(-1);
    }
    public void stop(){
        indexMotor.setPower(0);
    }
    public void slowFeed(){
        indexMotor.setPower(0.5);
    }
}
