package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intakeMotor;
    public Intake (HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void take(Gamepad gamepad){
        if(gamepad.right_trigger > 0.01)
            intakeMotor.setPower(gamepad.right_trigger);
        else if (gamepad.left_trigger > 0.01)
            intakeMotor.setPower(-gamepad.left_trigger);
        else
            stop();
    }
    public void autoTake(){
        intakeMotor.setPower(1);
    }
    public void leaveGate(){
        intakeMotor.setPower(0.65);
    }
    public void spit(){
        intakeMotor.setPower(-1);
    }
    public void stop(){
        intakeMotor.setPower(0);
    }
}
