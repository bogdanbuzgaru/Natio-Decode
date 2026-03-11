package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.MotorCommands;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.movement.Movement;

@TeleOp
public class TuneOdos extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private Movement movement;
    private double x = 3.5433070866, y = -4.2519685039;

    private double step = 1;

    public void init(){
        movement = new Movement(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(x, y, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                0, 0,
                AngleUnit.DEGREES,
                90
        ));
        pinpoint.recalibrateIMU();
    }

    public void loop(){
        movement.movementLoop(gamepad1);

        if(gamepad1.dpadDownWasPressed()){
            step /= 10;
        }else if(gamepad1.dpadUpWasPressed()){
            step *= 10;
        }

        if(gamepad1.squareWasPressed()){
            x -= step;
        }else if (gamepad1.circleWasPressed()) {
            x += step;
        }

        if(gamepad1.leftBumperWasPressed()){
            y -= step;
        }else if (gamepad1.rightBumperWasPressed()) {
            y += step;
        }

        if(gamepad1.crossWasPressed()){
            pinpoint.setOffsets(x, y, DistanceUnit.INCH);
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    0, 0,
                    AngleUnit.DEGREES,
                    90
            ));
            pinpoint.recalibrateIMU();
        }


        pinpoint.update();
        telemetry.addData("X offset", x);
        telemetry.addData("Y offset", y);
        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading", pinpoint.getHeading(AngleUnit.DEGREES));
    }
}
