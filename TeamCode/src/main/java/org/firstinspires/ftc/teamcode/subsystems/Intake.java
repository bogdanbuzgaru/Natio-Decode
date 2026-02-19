package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intakeMotor;
    public Intake (HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void take(double val){
        if (val > 0.1) {
            intakeMotor.setPower(1);
        }else{
            stop();
        }
    }
    public void spit(){
        intakeMotor.setPower(-1);
    }
    public void stop(){
        intakeMotor.setPower(0);
    }
}
