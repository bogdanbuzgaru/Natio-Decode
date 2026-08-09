package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Serpo extends OpMode {

    private Servo servo;
    private double position = 0;
    private double step = 0.01, step2 = 0.1, actual = 0.1;
    public void init(){
        servo = hardwareMap.get(Servo.class, "hood");
        servo.setPosition(1);
    }
    public void loop(){
        if(gamepad1.dpadUpWasPressed()){
            position += 0.1;
        }else if (gamepad1.dpadDownWasPressed()){
            position -= 0.1;
        }
        servo.setPosition(position);
    }
}