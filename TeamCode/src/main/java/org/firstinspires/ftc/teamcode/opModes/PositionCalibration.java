package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.limelight.Limelight;
import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


@TeleOp
public class PositionCalibration  extends OpMode {
    public static Follower follower;
    private Movement movement;
    private Lift lift;
    private Limelight limelight;
    private Turret turret;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 0, Math.toRadians(0)));
        limelight = new Limelight(hardwareMap, "limelight");
        movement = new Movement(hardwareMap);
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        follower.update();
    }
    public void loop(){
        follower.update();
//        limelight.update();
        turret.goNeutral();
//        movement.movementLoop(gamepad1);
//        movement.movementFieldCentric(gamepad2, follower.getPose().getHeading(), true);
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Yaw: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Is detecting", limelight.hasTarget());
        telemetry.addData("where to go", limelight.choice(limelight.getSelectedPath(), true));
    }
}
