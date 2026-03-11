package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * FLYWHEEL CALIBRATION OPMODE
 *
 * This OpMode helps you collect data to calibrate your flywheel speed.
 *
 * INSTRUCTIONS:
 * 1. Set hood to 45 degrees (measure with protractor)
 * 2. Use DPAD UP/DOWN to adjust flywheel speed
 * 3. Press A to record current speed
 * 4. Shoot ball and measure horizontal distance
 * 5. Enter distance below and it calculates launch speed
 * 6. Repeat at 6 different speeds
 * 7. Export data to Excel and create trendline
 *
 * SPEEDS TO TEST:
 * - 500 ticks/sec
 * - 700 ticks/sec
 * - 900 ticks/sec
 * - 1100 ticks/sec
 * - 1300 ticks/sec
 * - 1500 ticks/sec
 */
@TeleOp(name = "Flywheel Calibration", group = "Tuning")
public class FlywheelCalibration extends OpMode {

    private Shooter shooter;
    private Movement movement;
    private Index index;
    private Intake intake;
    private GoBildaPinpointDriver pinpoint;
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        movement = new Movement(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index (hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(3.64173228, -5.5511811, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                7.2440944808, 6.43700786745,
                AngleUnit.DEGREES,
                90
        ));
        pinpoint.recalibrateIMU();
    }

    @Override
    public void loop() {
        pinpoint.update();
        index.feed(gamepad1);
        intake.take(gamepad1);
        shooter.setHoodPosition(gamepad1);
        movement.movementLoop(gamepad1);
        telemetry.addData("Hood Position", shooter.getHoodPosition());
        telemetry.addData("Ticks", shooter.getTicks());
        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
    }

    @Override
    public void stop() {
    }
}