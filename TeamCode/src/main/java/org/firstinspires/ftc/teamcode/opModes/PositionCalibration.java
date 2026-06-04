package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


@TeleOp
public class PositionCalibration {
    public static Follower follower;
    private Movement movement;
    private Lift lift;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(7.2440944808, 7.08661417, Math.toRadians(90)));
        movement = new Movement(hardwareMap);
        lift = new Lift(hardwareMap);
        follower.update();
    }
    public void loop(){
        follower.update();
        movement.movementLoop(gamepad1);
        movement.movementFieldCentric(gamepad2, follower.getPose().getHeading());
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Yaw: ", Math.toDegrees(follower.getPose().getHeading()));
    }
}
