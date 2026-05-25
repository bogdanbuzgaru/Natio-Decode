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
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Index;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.io.File;

@Autonomous
public class BlueClose extends OpMode {

    public enum AutoState {
        SHOOT_FIRST,
        PREPARE_SECOND,
        SECOND_ROW,
        SHOOT_SECOND,
        GO_TO_GOAL,
        GO_BACK,
        SHOOT_THIRD,
        PREPARE_LAST_ROW,
        LAST_ROW,
        SHOOT_LAST,
        FIRST_ROW,
        SHOOT,
        PARK
    }

    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Index index;
    private StateMachine<AutoState> fsm = new StateMachine<>(AutoState.SHOOT_FIRST);
    private Follower follower;
    private boolean isShooting = false;
    private ElapsedTime pathTimer = new ElapsedTime();
    private Paths paths;
    private Position pos;
    private boolean repeat = true;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Mirrored Start Pose
        // X: 144 - (120 - 9.732) = 33.732
        // Y: 144 - (144 - 6.673) = 137.327
        follower.setStartingPose(new Pose(33.7322834608, 137.326771661, Math.toRadians(180)));
        paths = new Paths(follower);
        pos = new Position(follower.getPose());
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        pos.setBlue();
    }

    @Override
    public void start() {
        setUp();
        fsm.init();
        shooter.lowerBarrier();
    }

    @Override
    public void loop() {
        follower.update();
        fsm.update();
        turret.setAutoBlue();
        shooter.setTicks(1450);
        shooter.update();
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        String data = pose.getX() + "\n" + pose.getY() + "\n" + Math.toDegrees(pose.getHeading());
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, data);
    }

    private AutoState handleShoot(AutoState nextState, long durationMs, boolean change) {
        repeat = change;
        if (!isShooting) {
            pathTimer.reset();
            isShooting = true;
            shooter.raiseBarrier();
        } else {
            index.autoFeed();
            if (pathTimer.milliseconds() > durationMs) {
                isShooting = false;
                return nextState;
            }
        }
        return null;
    }

    private void setUp() {
        fsm.onStateEnter(AutoState.SHOOT_FIRST, () -> {
            follower.followPath(paths.SHOOT_FIRST);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_FIRST, () -> {
            intake.autoTake();
            if (!follower.isBusy()) return handleShoot(AutoState.PREPARE_SECOND, 900, true);
            return null;
        });

        fsm.onStateEnter(AutoState.PREPARE_SECOND, () -> {
            follower.followPath(paths.PREPARE_SECOND);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.PREPARE_SECOND, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) return AutoState.SECOND_ROW;
            return null;
        });

        fsm.onStateEnter(AutoState.SECOND_ROW, () -> {
            follower.followPath(paths.SECOND_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SECOND_ROW, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) return AutoState.SHOOT_SECOND;
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT_SECOND, () -> {
            follower.followPath(paths.SHOOT_SECOND);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_SECOND, () -> {
            intake.autoTake();
            if (!follower.isBusy()) return handleShoot(AutoState.GO_TO_GOAL, 700, true);
            return null;
        });

        fsm.onStateEnter(AutoState.GO_TO_GOAL, () -> {
            follower.followPath(paths.GO_TO_GOAL);
            shooter.lowerBarrier();
            pathTimer.reset();
            return null;
        });
        fsm.onStateUpdate(AutoState.GO_TO_GOAL, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) return AutoState.GO_BACK;
            return null;
        });

        fsm.onStateEnter(AutoState.GO_BACK, () -> {
            follower.followPath(paths.GO_BACK);
            shooter.lowerBarrier();
            pathTimer.reset();
            return null;
        });
        fsm.onStateUpdate(AutoState.GO_BACK, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy() && pathTimer.milliseconds() >= 3400) return AutoState.SHOOT_THIRD;
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT_THIRD, () -> {
            follower.followPath(paths.SHOOT_THIRD);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_THIRD, () -> {
            intake.autoTake();
            if (!follower.isBusy() && !repeat) {
                return handleShoot(AutoState.PREPARE_LAST_ROW, 700, false);
            } else if (!follower.isBusy() && repeat) {
                return handleShoot(AutoState.GO_TO_GOAL, 700, false);
            }
            return null;
        });

        fsm.onStateEnter(AutoState.PREPARE_LAST_ROW, () -> {
            follower.followPath(paths.PREPARE_LAST_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.PREPARE_LAST_ROW, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) return AutoState.LAST_ROW;
            return null;
        });

        fsm.onStateEnter(AutoState.LAST_ROW, () -> {
            follower.followPath(paths.LAST_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.LAST_ROW, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) return AutoState.SHOOT_LAST;
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT_LAST, () -> {
            follower.followPath(paths.SHOOT_LAST);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_LAST, () -> {
            intake.autoTake();
            if (!follower.isBusy()) return handleShoot(AutoState.FIRST_ROW, 700, false);
            return null;
        });

        fsm.onStateEnter(AutoState.FIRST_ROW, () -> {
            follower.followPath(paths.FIRST_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.FIRST_ROW, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) return AutoState.SHOOT;
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT, () -> {
            follower.followPath(paths.SHOOT);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT, () -> {
            intake.autoTake();
            if (!follower.isBusy()) return handleShoot(AutoState.PARK, 700, false);
            return null;
        });

        fsm.onStateEnter(AutoState.PARK, () -> {
            follower.followPath(paths.PARK);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.PARK, () -> {
            intake.autoTake();
            return null;
        });
    }

    public static class Paths {
        public PathChain SHOOT_FIRST, PREPARE_SECOND, SECOND_ROW, SHOOT_SECOND, GO_TO_GOAL,
                GO_BACK, SHOOT_THIRD, PREPARE_LAST_ROW, LAST_ROW, SHOOT_LAST,
                FIRST_ROW, SHOOT, PARK;

        public Paths(Follower follower) {
            SHOOT_FIRST = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(33.7322834608, 126.307087 - 1.968504), //1.968504 (intake extention, supercycle)
                            new Pose(54.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            PREPARE_SECOND = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 90.000),
                            new Pose(44.000, 60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SECOND_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(44.000, 60.000),
                            new Pose(10.000, 65.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SHOOT_SECOND = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(10.000, 65.000),
                            new Pose(29.000, 70.000),
                            new Pose(45.000, 72.000),
                            new Pose(50.000, 74.000),
                            new Pose(52.000, 79.000),
                            new Pose(53.000, 81.000),
                            new Pose(54.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GO_TO_GOAL = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 90.000),
                            new Pose(41.000, 57.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            GO_BACK = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(41.000, 57.000),
                            new Pose(10.000, 59.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(137))
                    .build();

            SHOOT_THIRD = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(10.000, 59.000),
                            new Pose(14.000, 60.000),
                            new Pose(29.000, 62.000),
                            new Pose(47.000, 70.000),
                            new Pose(51.000, 79.000),
                            new Pose(54.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            PREPARE_LAST_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 90.000),
                            new Pose(38.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            LAST_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(38.000, 36.000),
                            new Pose(10.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SHOOT_LAST = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 36.000),
                            new Pose(54.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            FIRST_ROW = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 90.000),
                            new Pose(16.000, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SHOOT = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(16.000, 84.000),
                            new Pose(54.000, 90.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            PARK = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(54.000, 90.000),
                            new Pose(16.000, 93.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }
}