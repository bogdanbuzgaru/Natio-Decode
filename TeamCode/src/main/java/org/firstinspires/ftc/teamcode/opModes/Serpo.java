//package org.firstinspires.ftc.teamcode.opModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//@TeleOp
//public class Serpo extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Servo servo = hardwareMap.servo.get("Servo");
//
//        double position = 0.5;
//
//        // edge detection variables
//        boolean lastUp = false;
//        boolean lastDown = false;
//
//        servo.setPosition(position);
//
//        telemetry.addLine("Servo Test Ready");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while(opModeIsActive()){
//
//            boolean up = gamepad1.dpad_up;
//            boolean down = gamepad1.dpad_down;
//
//            // only move once per press
//            if(up && !lastUp){
//                position += 0.01;
//            }
//
//            if(down && !lastDown){
//                position -= 0.01;
//            }
//
//            lastUp = up;
//            lastDown = down;
//
//            // presets
//            if(gamepad1.a) position = 0.0;
//            if(gamepad1.b) position = 0.5;
//            if(gamepad1.y) position = 1.0;
//
//            position = Range.clip(position, 0.0, 1.0);
//
//            servo.setPosition(position);
//
//            telemetry.addData("Servo Position", position);
//            telemetry.update();
//
//            sleep(20);
//        }
//    }
//}