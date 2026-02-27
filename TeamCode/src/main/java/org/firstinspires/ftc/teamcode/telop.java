package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class telop extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{

        DcMotor index = hardwareMap.dcMotor.get("index");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        index.setPower(0);
        intake.setPower(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            index.setPower(1);
            intake.setPower(1);
        }
    }
}