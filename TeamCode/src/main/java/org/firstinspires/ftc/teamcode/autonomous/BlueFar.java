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

@Autonomous(name = "BlueFar")
public class BlueFar extends OpMode {

    public enum AutoStates {
        PREPARE,
        TAKE_HUMAN,
        GO_SHOOT_HU,
        CENTER_LAST_ROW,
        TAKE_LAST_ROW,
        GO_SHOOT_LAST_ROW,
        ROTATE,
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
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Index index;
    private int number = 0;
    private boolean change = false;
    private boolean repeat = true;
    private Position pos;
    private Limelight limelight;
    private int choice;
    private ElapsedTime autoTimer;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(53.690, 9.100, Math.toRadians(180)));
        pos = new Position(follower.getPose());
        pos.setBlue();
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
        if (!change)
            turret.angleToPos(-78.82);
        else {
            turret.angleToPos(-75.68);
        }
        shooter.setTicks(1500, false, false);
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

    private AutoStates handleShoot(AutoStates nextState, long durationMs, boolean change) {
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

    private void setUp() {
        fsm.onStateEnter(AutoStates.PREPARE, () -> {
            pathTimer.reset();
            return null;
        });

        fsm.onStateUpdate(AutoStates.PREPARE, () -> {
            if (pathTimer.milliseconds() > 2800) {
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
            change = true;
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
            if (!follower.isBusy()) {
                return handleShoot(AutoStates.CENTER_LAST_ROW, 700, true);
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
            if (!follower.isBusy()) {
                return handleShoot(AutoStates.WAIT, 700, true);
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.ROTATE, () -> {
            follower.followPath(paths.ROTATE);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.ROTATE, () -> {
            intake.autoTake();
            int choice = limelight.choice(limelight.getSelectedPath(), true);
            if (!follower.isBusy() && (choice == 2 || choice == 1)) {
                return AutoStates.TAKE_RANDOM;
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
            return null;
        });

        fsm.onStateUpdate(AutoStates.WAIT, () -> {
            intake.autoTake();
            int choice = limelight.choice(limelight.getSelectedPath(), true);
            if (!follower.isBusy()) {
                if (autoTimer.milliseconds() > 28000) {
                    return handleShoot(AutoStates.PARK, 700, true);
                } else if (number < 5 && choice == 1) {
                    AutoStates next = handleShoot(AutoStates.TAKE_HUMAN, 700, true);
                    if (next != null) number++;
                    return next;
                } else if (number < 5 && choice == 2) {
                    AutoStates next = handleShoot(AutoStates.TAKE_HUMAN, 700, true);
                    if (next != null) number++;
                    return next;
                } else if (autoTimer.milliseconds() < 27000 && choice == 0) {
                    AutoStates next = handleShoot(AutoStates.ROTATE, 700, true);
                    if (next != null) number++;
                    return next;
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
                ROTATE, TAKE_RANDOM, SHOOT_RANDOM ;

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
                            new Pose(59.000, 15.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            CENTER_LAST_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.000, 15.000),
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
                            new Pose(59.000, 15.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ROTATE = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(59.000, 15.000),
                            new Pose(52.000, 16.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            TAKE_RANDOM = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(52.000, 16.000),
                            new Pose (33.000, 35.000),
                            new Pose(10.000, 44.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            SHOOT_RANDOM = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 44.000),
                            new Pose(59.000, 15.000)
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