package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
        SECOND_ROW,
        SHOOT_SECOND,
        GO_TO_GOAL,
        GO_BACK,
        SHOOT_THIRD,
        LAST_ROW,
        SHOOT_LAST,
        FIRST_ROW,
        SHOOT,
        PARK
    }

    private Position position;
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Index index;
    private StateMachine<AutoState> fsm = new StateMachine<AutoState>(AutoState.SHOOT_FIRST);
    private Follower follower;
    private boolean isShooting = false;
    private ElapsedTime pathTimer = new ElapsedTime();
    private Paths paths;

    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24 + 9.7322834608, 140 - 6.67322833945, Math.toRadians(180)));
        paths = new Paths(follower);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        index = new Index(hardwareMap);
        position = new Position(new Pose2D(DistanceUnit.INCH, 24 + 9.7322834608, 140 - 6.67322833945, AngleUnit.DEGREES, 180));
    }

    public void start() {
        setUp();
        fsm.init();
        shooter.lowerBarrier();
    }

    public void loop() {
        follower.update();
        fsm.update();
        position.update(new Pose2D(DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.DEGREES,
                Math.toDegrees(follower.getHeading())));

        turret.setTargetAngle(position.getTargetAngle());
//        turret.setHeading(position.getHeading());
        turret.update();

        shooter.setTicks(1300);
        shooter.update();
    }

    public void stop() {
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(pose.getHeading());

        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, xPose + "\n" + yPose + "\n" + heading);
    }

    private AutoState handleShoot(AutoState nextState, long durationMs) {
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
            if (!follower.isBusy()) {
                return handleShoot(AutoState.SECOND_ROW, 700);
            }
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
            if (!follower.isBusy()) {
                return AutoState.SHOOT_SECOND;
            }
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT_SECOND, () -> {
            follower.followPath(paths.SHOOT_SECOND);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_SECOND, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoState.GO_TO_GOAL, 700);
            }
            return null;
        });

        fsm.onStateEnter(AutoState.GO_TO_GOAL, () -> {
            follower.followPath(paths.GO_TO_GOAL);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.GO_TO_GOAL, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) {
                return AutoState.GO_BACK;
            }
            return null;
        });

        fsm.onStateEnter(AutoState.GO_BACK, () -> {
            follower.followPath(paths.GO_BACK);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.GO_BACK, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) {
                return AutoState.SHOOT_THIRD;
            }
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT_THIRD, () -> {
            follower.followPath(paths.SHOOT_THIRD);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_THIRD, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoState.LAST_ROW, 700);
            }
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
            if (!follower.isBusy()) {
                return AutoState.SHOOT_LAST;
            }
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT_LAST, () -> {
            follower.followPath(paths.SHOOT_LAST);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_LAST, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoState.FIRST_ROW, 700);
            }
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
            if (!follower.isBusy()) {
                return AutoState.SHOOT;
            }
            return null;
        });

        fsm.onStateEnter(AutoState.SHOOT, () -> {
            follower.followPath(paths.SHOOT);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoState.PARK, 700);
            }
            return null;
        });

        fsm.onStateEnter(AutoState.PARK, () -> {
            follower.followPath(paths.PARK);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.PARK, () -> {
            intake.autoTake();
            // if(!follower.isBusy()) {
            //     requestOpModeStop();
            // }
            return null;
        });
    }

    public static class Paths {
        public PathChain SHOOT_FIRST;
        public PathChain SECOND_ROW;
        public PathChain SHOOT_SECOND;
        public PathChain GO_TO_GOAL;
        public PathChain GO_BACK;
        public PathChain SHOOT_THIRD;
        public PathChain LAST_ROW;
        public PathChain SHOOT_LAST;
        public PathChain FIRST_ROW;
        public PathChain SHOOT;
        public PathChain PARK;

        private double actualPositionX(double value) {
            return value + 9.7322834608;
        }

        private double actualPositionY(double value) {
            return value - 6.67322833945;
        }

        public Paths(Follower follower) {
            SHOOT_FIRST = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(actualPositionX(24.000), actualPositionY(140.000)),
                                    new Pose(34.000, 130.000),
                                    new Pose(49.000, 105.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());

            SECOND_ROW = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(55.000, 74.000),
                                    new Pose(47.000, 66.000),
                                    new Pose(32.000, 62.000),
                                    new Pose(17.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());

            SHOOT_SECOND = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.000, 60.000),
                                    new Pose(29.000, 60.000),
                                    new Pose(45.000, 68.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());

            GO_TO_GOAL = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(53.000, 76.000),
                                    new Pose(40.000, 67.000),
                                    new Pose(24.000, 63.000),
                                    new Pose(15.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build());

            GO_BACK = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.000, 63.000),
                                    new Pose(14.500, 61.000),
                                    new Pose(14.000, 58.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(137))
                    .build());

            SHOOT_THIRD = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.000, 58.000),
                                    new Pose(14.000, 65.000),
                                    new Pose(29.000, 72.000),
                                    new Pose(47.000, 78.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build());

            LAST_ROW = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(56.000, 68.000),
                                    new Pose(51.000, 54.000),
                                    new Pose(36.000, 40.000),
                                    new Pose(10.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());

            SHOOT_LAST = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 36.000),
                                    new Pose(24.000, 38.000),
                                    new Pose(39.000, 48.000),
                                    new Pose(50.000, 64.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());

            FIRST_ROW = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(44.000, 86.000),
                                    new Pose(30.000, 85.000),
                                    new Pose(16.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());

            SHOOT = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.000, 84.000),
                                    new Pose(30.000, 82.000),
                                    new Pose(44.000, 83.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());

            PARK = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(44.000, 87.000),
                                    new Pose(32.000, 89.000),
                                    new Pose(24.000, 89.000),
                                    new Pose(16.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build());
        }
    }
}