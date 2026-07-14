package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Serpo extends OpMode {

    private DcMotorEx servo;
    private double position = 0;
    private double step = 0.01, step2 = 0.1, actual = 0.1;
    public void init(){
        servo = hardwareMap.get(DcMotorEx.class, "flywheel2");
    }
    public void loop(){
        servo.setPower(0.5);
    }
}