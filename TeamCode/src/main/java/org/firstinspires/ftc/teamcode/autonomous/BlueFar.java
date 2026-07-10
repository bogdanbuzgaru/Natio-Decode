package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.limelight.Limelight;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.io.File;

@Autonomous
public class BlueFar extends OpMode {

    public enum AutoStates {
        PREPARE,
        TAKE_HUMAN,
        GO_SHOOT_HU,
        CENTER_LAST_ROW,
        TAKE_LAST_ROW,
        GO_SHOOT_LAST_ROW,
        REPEAT,
        TAKE_RANDOM,
        SHOOT_RANDOM,
        WAIT,
        PARK
    }

    private StateMachine<AutoStates> fsm = new StateMachine<>(AutoStates.PREPARE);
    private Follower follower;
    private Paths paths;
    private boolean isShooting = false;
    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime(); // Added dedicated timer for handleShoot
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Index index;
    private int number = 0;
    private Position pos;
    private Limelight limelight;
    private ElapsedTime autoTimer;
    private boolean skip = false;
    private boolean add = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Updated to mirrored starting pose
        follower.setStartingPose(new Pose(53.690, 9.100, Math.toRadians(180)));
        pos = new Position(follower.getPose());
        pos.setRed();
        limelight = new Limelight(hardwareMap, "limelight");
        paths = new Paths(follower);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        autoTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        setUp();
        fsm.init();
        shooter.lowerBarrier();
        autoTimer.reset();
    }

    @Override
    public void loop() {
        limelight.update();
        follower.update();
        pos.update(follower.getPose());
        fsm.update();
//        turret.angleToPos(85.82);
        turret.setTargetAngle(pos.target()); //+ angle if needed
        turret.setHeading(Math.toDegrees(follower.getHeading()), false);
        turret.update();
        shooter.setTicks(1590, false, false);
        shooter.updateMotor();
        index.normalIndex();
        telemetry.addData("has target", limelight.hasTarget());
        telemetry.addData("Object", limelight.choice(limelight.getSelectedPath(), true));
    }

    @Override
    public void stop() {
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(Math.toDegrees(pose.getHeading()));

        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, 0 + "\n" + xPose + "\n" + yPose + "\n" + heading);
    }

    // Cleaned up unused boolean parameter, updated to use shootTimer
    private AutoStates handleShoot(AutoStates nextState, long durationMs) {
        if (!isShooting) {
            shootTimer.reset();
            isShooting = true;
            shooter.raiseBarrier();
        } else {
            index.autoFeed();
            intake.autoTake();
            if (shootTimer.milliseconds() > durationMs) {
                isShooting = false;
                return nextState;
            }
        }
        return null;
    }

    private void setUp() {
        fsm.onStateEnter(AutoStates.PREPARE, () -> {
            pathTimer.reset();
            return null;
        });

        fsm.onStateUpdate(AutoStates.PREPARE, () -> {
            if (pathTimer.milliseconds() > 1400) {
                index.autoFeedFar();
                intake.autoTake();
                return handleShoot(AutoStates.TAKE_HUMAN, 600);
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
            if (!follower.isBusy()) {
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
            if (!follower.isBusy() && !skip) {
                return handleShoot(AutoStates.CENTER_LAST_ROW, 600);
            }else if (!follower.isBusy() && skip){
                if (number < 6 && number % 4 == 0) {
                    return handleShoot(AutoStates.REPEAT, 600);
                }else if (number < 6) {
                    add = true;
                    return handleShoot(AutoStates.TAKE_RANDOM, 600);
                }else if (!follower.isBusy()){
                    return handleShoot(AutoStates.PARK, 600);
                }
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.CENTER_LAST_ROW, () -> {
            follower.followPath(paths.CENTER_LAST_ROW);
            shooter.lowerBarrier();
            skip = true;
            if (add == true){
                number++;
            }
            return null;
        });

        fsm.onStateUpdate(AutoStates.CENTER_LAST_ROW, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
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
            if (!follower.isBusy()) {
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
            if (!follower.isBusy() && number < 5) {
                return handleShoot(AutoStates.TAKE_HUMAN, 500);
            }else if (!follower.isBusy()){
                return AutoStates.PARK;
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.REPEAT, () -> {
            follower.followPath(paths.REPEAT);
            shooter.lowerBarrier();
            number++;
            return null;
        });
        fsm.onStateUpdate(AutoStates.REPEAT, () -> {
            intake.autoTake();
//            int choice = limelight.choice(limelight.getSelectedPath(), true);
//            if(autoTimer.seconds() > 24){
//                return AutoStates.WAIT;
//            }
            if (!follower.isBusy()) {
                return AutoStates.GO_SHOOT_HU;
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.TAKE_RANDOM, () -> {
            follower.followPath(paths.TAKE_RANDOM);
            shooter.lowerBarrier();
            return null;
        });

        fsm.onStateUpdate(AutoStates.TAKE_RANDOM, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return AutoStates.SHOOT_RANDOM;
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.SHOOT_RANDOM, () -> {
            follower.followPath(paths.SHOOT_RANDOM);
            shooter.lowerBarrier();
            return null;
        });

        fsm.onStateUpdate(AutoStates.SHOOT_RANDOM, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return AutoStates.WAIT;
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.WAIT, () -> {
            shooter.lowerBarrier();
            number++;
            return null;
        });

        fsm.onStateUpdate(AutoStates.WAIT, () -> {
            intake.autoTake();
            if (!follower.isBusy() && !skip) {
                return handleShoot(AutoStates.CENTER_LAST_ROW, 600);
            }else if (!follower.isBusy() && skip){
                if (number < 6 && number % 4 == 0) {
                    return handleShoot(AutoStates.REPEAT, 600);
                }else if (number < 6) {
                    add = true;
                    return handleShoot(AutoStates.TAKE_RANDOM, 600);
                }else if (!follower.isBusy()){
                    return handleShoot(AutoStates.PARK, 600);
                }
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.PARK, () -> {
            follower.followPath(paths.PARK);
            return null;
        });

        fsm.onStateUpdate(AutoStates.PARK, () -> null);
    }

    public static class Paths {
        public PathChain TAKE_HUMAN, GO_SHOOT_HU, PARK, CENTER_LAST_ROW, TAKE_LAST_ROW, GO_SHOOT_LAST_ROW,
                REPEAT, TAKE_RANDOM, SHOOT_RANDOM ;

        public Paths(Follower follower) {
            TAKE_HUMAN = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(53.690, 9.100),
                            new Pose(10.000, 11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GO_SHOOT_HU = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 11.000),
                            new Pose(55.000, 15.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            CENTER_LAST_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.000, 15.000),
                            new Pose(39.200, 35.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            TAKE_LAST_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(39.200, 35.000),
                            new Pose(17.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GO_SHOOT_LAST_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(17.000, 36.000),
                            new Pose(55.000, 15.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            REPEAT = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.000, 15.000),
                            new Pose(10.000, 11.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            TAKE_RANDOM = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(52.000, 16.000),
                            new Pose (23.100, -2.500),
                            new Pose(10.000, -8.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SHOOT_RANDOM = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, -8.000),
                            new Pose(55.000, 15.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            PARK = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(50.000, 15.000),
                            new Pose(33.000, 16.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}