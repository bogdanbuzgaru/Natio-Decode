package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Serpo extends OpMode {

    private Servo servo;
    private double position = 0;
    private double step = 0.01, step2 = 0.1, actual = 0.1;
    public void init(){
        servo = hardwareMap.get(Servo.class, "barrier");
    }
    public void loop(){
         if(gamepad1.squareWasPressed()){
             actual = step;
         }else if (gamepad1.circleWasPressed()){
             actual = step2;
         }
         if(gamepad1.dpadUpWasPressed()){
             position = Math.min(1.00, position + actual);
         }else if (gamepad1.dpadDownWasPressed()){
             position = Math.max(0.0, position - actual);
         }
         servo.setPosition(position);
         telemetry.addData("Position", position);
         telemetry.addData("Servo pos", servo.getPosition());
         telemetry.addData("Step", actual);
    }
}