package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

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
public class RedNeutral extends OpMode {
    public enum AutoStates {
        SHOOT_PRELOAD,
        TAKE_FIRST,
        SHOOT_FIRST,
        TAKE_SECOND,
        SHOOT_SECOND,
        TAKE_THIRD,
        SHOOT_THIRD,
        TAKE_RANDOM,
        SHOOT_RANDOM,
        PARK
    }

    private StateMachine<AutoStates> fsm = new StateMachine<>(AutoStates.SHOOT_PRELOAD);
    private Turret turret;
    private Shooter shooter;
    private Intake intake;
    private Index index;
    private Follower follower;
    private Position pos;
    private boolean isShooting = false;
    private ElapsedTime pathTimer = new ElapsedTime();
    private Paths paths;

    private final long SHOOT_FIRST_MS = 560, SHOOT_MS = 620;

    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(75.69912406215, -115.056875942, Math.toRadians(-125)));
        paths = new Paths(follower);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake (hardwareMap);
        index = new Index(hardwareMap);
        pos = new Position(follower.getPose());
        pos.setRed();
    }

    public void start() {
        setUp();
        fsm.init();
        shooter.lowerBarrier();
    }
    public void loop(){
        follower.update();
        fsm.update();
        pos.update(follower.getPose());
        index.normalIndex();
        turret.angleToPos(-120);     //TODO change angle
//        turret.setTargetAngle(pos.target()); //+ angle if needed        //TODO changed from target to getTargetAngle or smth
//        turret.setHeading(Math.toDegrees(follower.getHeading()), true);
        turret.update();
        shooter.setTicks(1230, false, false);   //TODO change ticks
        shooter.update();

        telemetry.addData("Angle", turret.getPosition1());
    }
    public void stop() {
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(Math.toDegrees(pose.getHeading()));
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, 1 + "\n" + xPose + "\n" + yPose + "\n" + heading);

    }

    private void setUp() {
        fsm.onStateEnter(AutoStates.SHOOT_PRELOAD, () -> {
            follower.followPath(paths.SHOOT_PRELOAD);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.SHOOT_PRELOAD, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoStates.TAKE_FIRST, SHOOT_FIRST_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.TAKE_FIRST, () -> {
            follower.followPath(paths.TAKE_FIRST);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.TAKE_FIRST, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return AutoStates.SHOOT_FIRST;
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.SHOOT_FIRST, () -> {
            follower.followPath(paths.SHOOT_FIRST);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.SHOOT_FIRST, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoStates.TAKE_SECOND, SHOOT_MS);
            }
            return null;
        });

        fsm.onStateEnter(AutoStates.TAKE_SECOND, () -> {
            follower.followPath(paths.TAKE_SECOND);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.TAKE_SECOND, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return AutoStates.SHOOT_SECOND;
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.SHOOT_SECOND, () -> {
            follower.followPath(paths.SHOOT_SECOND);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.SHOOT_SECOND, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoStates.TAKE_THIRD, SHOOT_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.TAKE_THIRD, () -> {
            follower.followPath(paths.TAKE_THIRD);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.TAKE_THIRD, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return AutoStates.SHOOT_THIRD;
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.SHOOT_THIRD, () -> {
            follower.followPath(paths.SHOOT_THIRD);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.SHOOT_THIRD, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoStates.TAKE_RANDOM, SHOOT_MS);
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
                return handleShoot(AutoStates.PARK, SHOOT_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoStates.PARK, () -> {
            follower.followPath(paths.PARK);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoStates.PARK, () -> {
            intake.autoTake();
            if (!follower.isBusy()) {
                stop();
            }
            return null;
        });
    }
    private AutoStates handleShoot(AutoStates nextState, long durationMs) {
        if (!isShooting) {
            pathTimer.reset();
            isShooting = true;
            shooter.raiseBarrier();
        }
        index.autoFeed();
        if (pathTimer.milliseconds() > durationMs) {
            isShooting = false;
            return nextState;
        }
        return null;
    }
    public static class Paths {
        public PathChain  SHOOT_PRELOAD,
                TAKE_FIRST,
                SHOOT_FIRST,
                TAKE_SECOND,
                SHOOT_SECOND,
                TAKE_THIRD,
                SHOOT_THIRD,
                TAKE_RANDOM,
                SHOOT_RANDOM,
                PARK;
        public Paths (Follower follower){
            SHOOT_PRELOAD = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(75.69912406215, -115.056875942),
                                    new Pose(96.000, -96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-125), Math.toRadians(0))
                    .build()
            );
            TAKE_FIRST = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.000, -96.000),
                                    new Pose(105.000, -88.000),
                                    new Pose(127.000, -84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );
            SHOOT_FIRST = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.000, -84.000),
                                    new Pose(96.000, -84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );
            TAKE_SECOND = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.000, -84.000),
                                    new Pose(105.000, -66.000),
                                    new Pose(127.000, -60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );
            SHOOT_SECOND = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(127.000, -60.000),
                                    new Pose(105.000, -105.000),
                                    new Pose(96.000, -72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );
            TAKE_THIRD = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.000, -72.000),
                                    new Pose(105.000, -50.000),
                                    new Pose(127.000, -36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );
            SHOOT_THIRD = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.000, -36.000),
                                    new Pose(96.000, -72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );
            TAKE_RANDOM = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, -72.000),
                                    new Pose(125.500, -131.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build()
            );
            SHOOT_RANDOM = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.500, -131.000),
                                    new Pose(96.000, -96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build()
            );
            PARK = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, -96.000),
                                    new Pose(125.000, -88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );
        }
    }
}
