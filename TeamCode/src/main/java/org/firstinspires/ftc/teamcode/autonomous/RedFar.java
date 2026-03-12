package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.math.Sensor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.io.File;

@Autonomous
public class RedFar extends OpMode {

    public enum AutoStates{
        PREPARE,
        TAKE_HUMAN,
        GO_SHOOT_HU,
        CENTER_LAST_ROW,
        TAKE_LAST_ROW,
        GO_SHOOT_LAST_ROW,
        PARK;
    }
    private StateMachine<AutoStates> fsm = new StateMachine<>(AutoStates.PREPARE);
    private Follower follower;
    private Paths paths;
    private boolean isShooting = false;
    private ElapsedTime pathTimer = new ElapsedTime();
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Index index;
    private Position position;
    private int number = 0;
    private boolean repeat;
    private GoBildaPinpointDriver pinpoint;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87.000, 8.000));
        paths = new Paths(follower);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake (hardwareMap);
        index = new Index(hardwareMap);
//        position = new Position(new Pose2D(DistanceUnit.INCH,120.000 - 9.7322834608,
//                144.000 - 6.67322833945, AngleUnit.DEGREES, 0));
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setOffsets(3.5433070866, -4.2519685039, DistanceUnit.INCH);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        pinpoint.setPosition(new Pose2D(
//                DistanceUnit.INCH,
//                87.000, 8.000,
//                AngleUnit.DEGREES,
//                90
//        ));
//        pinpoint.recalibrateIMU();
//        position = new Position(new Pose2D(
//                DistanceUnit.INCH,
//                87.000, 8.000,
//                AngleUnit.DEGREES,
//                90
//        ));
    }
    public void start(){
        setUp();
        fsm.init();
        shooter.lowerBarrier();
    }
    public void loop(){
        follower.update();
        fsm.update();
//        position.update(new Pose2D(DistanceUnit.INCH,
//                follower.getPose().getX(),
//                follower.getPose().getY(),
//                AngleUnit.DEGREES,
//                Math.toDegrees(follower.getHeading())));
//        pinpoint.update();
//        position.update(pinpoint.getPosition());
        turret.setFarAuto();
//        turret.setTargetAngle(position.getTargetAngle());
//        turret.setOffsetAngle(position.offsetAngleRed(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), position.getTicks(6.89911, 1100.04194)));
//        turret.update();
        shooter.setTicks(2140);
        shooter.updateMotor();
    }
    public void stop(){
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(pose.getHeading());

        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, xPose + "\n" + yPose + "\n" + heading);

    }
    private AutoStates handleShoot(AutoStates nextState, long durationMs, boolean change) {
        repeat = change;
        if (!isShooting) {
            pathTimer.reset();
            isShooting = true;
            shooter.raiseBarrier();
        } else {
            index.autoFeed();
            intake.autoTake();
            if (pathTimer.milliseconds() > durationMs) {
                isShooting = false;
                return nextState;
            }
        }
        return null;
    }
    private void setUp(){
        fsm.onStateEnter(AutoStates.PREPARE, () -> {
            pathTimer.reset();
            return null;
        });
        fsm.onStateUpdate(AutoStates.PREPARE, () -> {
            if(pathTimer.milliseconds() > 2800){
                index.autoFeed();
                intake.autoTake();
                return handleShoot(AutoStates.TAKE_HUMAN, 700, true);
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.TAKE_HUMAN, () -> {
            follower.followPath(paths.TAKE_HUMAN);
            shooter.lowerBarrier();
            number++;
            return null;
        });
        fsm.onStateUpdate(AutoStates.TAKE_HUMAN, () -> {
            intake.autoTake();
            if(!follower.isBusy()){
                return AutoStates.GO_SHOOT_HU;
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.GO_SHOOT_HU, () -> {
            follower.followPath(paths.GO_SHOOT_HU);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.GO_SHOOT_HU, () -> {
            intake.autoTake();
            if(!follower.isBusy() && number < 2){
                return handleShoot(AutoStates.CENTER_LAST_ROW, 700, true);
            }else if(!follower.isBusy() && number < 5){
                return handleShoot(AutoStates.TAKE_HUMAN, 700, true);
            }else if (!follower.isBusy()){
                return AutoStates.PARK;
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.CENTER_LAST_ROW, () -> {
            follower.followPath(paths.CENTER_LAST_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.CENTER_LAST_ROW, () -> {
            intake.autoTake();
            if(!follower.isBusy()){
                return AutoStates.TAKE_LAST_ROW;
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.TAKE_LAST_ROW, () -> {
            follower.followPath(paths.TAKE_LAST_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.TAKE_LAST_ROW, () -> {
            intake.autoTake();
            if(!follower.isBusy()){
                return AutoStates.GO_SHOOT_LAST_ROW;
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.GO_SHOOT_LAST_ROW, () -> {
            follower.followPath(paths.GO_SHOOT_LAST_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.GO_SHOOT_LAST_ROW, () -> {
            intake.autoTake();
            if(!follower.isBusy() && number < 3){
                return handleShoot(AutoStates.TAKE_HUMAN, 700, true);
            }
            return null;

        });
        fsm.onStateEnter(AutoStates.PARK, () -> {
            follower.followPath(paths.PARK);
            return null;
        });
        fsm.onStateUpdate(AutoStates.PARK, () -> {
            return null;
        });

    }
    public static class Paths {
        public PathChain TAKE_HUMAN;
        public PathChain GO_SHOOT_HU;
        public PathChain PARK;
        public PathChain CENTER_LAST_ROW;
        public PathChain TAKE_LAST_ROW;
        public PathChain GO_SHOOT_LAST_ROW;


        public Paths(Follower follower) {
            TAKE_HUMAN = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.000, 8.000),

                                    new Pose(134.000, 9.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            GO_SHOOT_HU = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 9.000),

                                    new Pose(94.000, 8.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
            CENTER_LAST_ROW = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.500, 8.500),

                                    new Pose(102.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
            TAKE_LAST_ROW = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 36.000),

                                    new Pose(120.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
            GO_SHOOT_LAST_ROW = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.000, 36.000),

                                    new Pose(94.000, 8.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
            PARK = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 8.500),
                                new Pose (111.000, 16.000)
                        )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
