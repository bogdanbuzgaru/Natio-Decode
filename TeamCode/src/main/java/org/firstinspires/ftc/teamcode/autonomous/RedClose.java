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
public class RedClose extends OpMode {

    public enum AutoState {
        SHOOT_FIRST,
        PREPARE_SECOND,
        SECOND_ROW,
        SHOOT_SECOND,
        GO_TO_GOAL,
//        GO_BACK,
        SHOOT_GOAL,
//        PREPARE_LAST_ROW,
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
    private StateMachine<AutoState> fsm = new StateMachine<AutoState>(AutoState.SHOOT_FIRST);
    private Follower follower;
    private Position pos;
    private boolean isShooting = false;
    private ElapsedTime pathTimer = new ElapsedTime();
    private Paths paths;
    private boolean repeat = true;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(120.000 - 9.7322834608, 144.000 - 6.67322833945));
        paths = new Paths(follower);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake (hardwareMap);
        index = new Index(hardwareMap);
        pos = new Position(follower.getPose());
        pos.setRed();
//        position = new Position(new Pose2D(DistanceUnit.INCH,120.000 - 9.7322834608,
//                144.000 - 6.67322833945, AngleUnit.DEGREES, 0));
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

//        turret.setAuto();
        pos.update(follower.getPose());
        turret.setTargetAngle(pos.getTargetAngle());
        turret.setOffsetAngle(pos.getOffetAngle(follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent()));
        turret.update();
        shooter.setTicks(1480);
        shooter.update();
    }
    public void stop(){
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(Math.toDegrees(pose.getHeading()));

        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, xPose + "\n" + yPose + "\n" + heading);

    }
    private AutoState handleShoot(AutoState nextState, long durationMs, boolean change) {
        if (!isShooting) {
            pathTimer.reset();
            isShooting = true;
            shooter.raiseBarrier();
        } else {
            index.autoFeed();
            if (pathTimer.milliseconds() > durationMs) {
                isShooting = false;
                repeat = change;
                return nextState;
            }
        }
        return null;
    }
    private void setUp(){
        fsm.onStateEnter(AutoState.SHOOT_FIRST, () -> {
            follower.followPath(paths.SHOOT_FIRST);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_FIRST, () -> {
            intake.autoTake();
            if(!follower.isBusy()) {
                return handleShoot(AutoState.SECOND_ROW, 900, true);
            }
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
            if(!follower.isBusy()) {
                return AutoState.SECOND_ROW;
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
            if(!follower.isBusy()) {
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
            if(!follower.isBusy()) {
                return handleShoot(AutoState.GO_TO_GOAL, 700, true);
            }
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
            if(!follower.isBusy()) {
                return AutoState.SHOOT_GOAL;
            }
            return null;
        });
//        fsm.onStateEnter(AutoState.GO_BACK, () -> {
//            follower.followPath(paths.GO_BACK);
//            shooter.lowerBarrier();
//            pathTimer.reset();
//            return null;
//
//        });
//        fsm.onStateUpdate(AutoState.GO_BACK, () -> {
//            intake.autoTake();
//            index.autoFeed();
//            if(!follower.isBusy() && pathTimer.milliseconds() >= 3400) {
//                return AutoState.SHOOT_GOAL;
//            }
//            return null;
//        });
        fsm.onStateEnter(AutoState.SHOOT_GOAL, () -> {
            follower.followPath(paths.SHOOT_GOAL);
            shooter.lowerBarrier();
            return null;

        });
        fsm.onStateUpdate(AutoState.SHOOT_GOAL, () -> {
            intake.autoTake();
            if(!follower.isBusy() && !repeat) {
                return handleShoot(AutoState.LAST_ROW, 700, false);
            }else if (!follower.isBusy() && repeat){
                return handleShoot(AutoState.GO_TO_GOAL, 700, false);
            }
            return null;
        });
//        fsm.onStateEnter(AutoState.PREPARE_LAST_ROW, () -> {
//            follower.followPath(paths.PREPARE_LAST_ROW);
//            shooter.lowerBarrier();
//            return null;
//        });
//        fsm.onStateUpdate(AutoState.PREPARE_LAST_ROW, () -> {
//            intake.autoTake();
//            index.autoFeed();
//            if(!follower.isBusy()) {
//                return AutoState.LAST_ROW;
//            }
//            return null;
//        });
        fsm.onStateEnter(AutoState.LAST_ROW, () -> {
            follower.followPath(paths.LAST_ROW);
            shooter.lowerBarrier();
            return null;

        });
        fsm.onStateUpdate(AutoState.LAST_ROW, () -> {
            intake.autoTake();
            index.autoFeed();
            if(!follower.isBusy()) {
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
            if(!follower.isBusy()) {
                return handleShoot(AutoState.FIRST_ROW, 700, false);

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
            if(!follower.isBusy()) {
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
            if(!follower.isBusy()) {
                return handleShoot(AutoState.PARK, 700, false);
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
//            if(!follower.isBusy()) {
//                requestOpModeStop();
//            }
            return null;
        });


    }
    public static class Paths {
        public PathChain SHOOT_FIRST;
        public PathChain PREPARE_SECOND;
        public PathChain SECOND_ROW;
        public PathChain SHOOT_SECOND;
        public PathChain GO_TO_GOAL;
        public PathChain GO_BACK;
        public PathChain SHOOT_GOAL;
        public PathChain PREPARE_LAST_ROW;
        public PathChain LAST_ROW;
        public PathChain SHOOT_LAST;
        public PathChain FIRST_ROW;
        public PathChain SHOOT;
        public PathChain PARK;

        private double actualPositionX(double value){
            return value - 9.7322834608;
        }
        private double actualPositionY(double value){
            return value - 6.692913 - 1.968504;
        } //1.968504 (intake extension)
        public Paths(Follower follower) {
            SHOOT_FIRST = (follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(actualPositionX(120.000), actualPositionY(133.000)),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

//            PREPARE_SECOND = (follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(90.000, 90.000),
//                                    new Pose(100.000, 67.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build()
//            );

            SECOND_ROW = (follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(100.000,67.000),
//                                    new Pose(134.000, 65.000)
//                            )
                            new BezierCurve(
                                    new Pose(84.000, 84.000),
                                    new Pose(84.322, 51.681),
                                    new Pose(127.434, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );

            SHOOT_SECOND = (follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(134.000, 65.000),
                                            new Pose(115.000, 70.000),
                                            new Pose(99.000, 72.000),
                                            new Pose(94.000, 74.000),
                                            new Pose(92.000, 79.000),
                                            new Pose(91.000, 81.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            GO_TO_GOAL = (follower.pathBuilder().addPath(
//                                    new BezierLine(
//                                            new Pose(90.000, 90.000),
//                                            new Pose(103.000, 59.000)
//                                    )
                                new BezierLine(
                                        new Pose(84.000, 84.000),
                                        new Pose(129.000, 57.000)
                                )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                            .build()
            );

            GO_BACK = (follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(103.000, 59.000),
                                            new Pose(134.000, 65.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                            .build()
            );

            SHOOT_GOAL = (follower.pathBuilder().addPath(
                                    new BezierCurve(
//                                            new Pose(134.000, 59.000),
//                                            new Pose(130.000, 60.000),
//                                            new Pose(115.000, 62.000),
//                                            new Pose(97.000, 70.000),
//                                            new Pose(93.000, 79.000),
//                                            new Pose(90.000, 90.000)
                                            new Pose(129.000, 57.000),
                                            new Pose(100.255, 67.783),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                            .build()
            );
//            PREPARE_LAST_ROW = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(90.000, 90.000),
//                                    new Pose(106.000, 40.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
            LAST_ROW = (follower.pathBuilder().addPath(
//                                    new BezierLine(
//                                            new Pose (106.000, 40.000),
//                                            new Pose(134.000, 40.000)
//                                    )
                                new BezierCurve(
                                        new Pose(84.000, 84.000),
                                        new Pose(88.633, 23.549),
                                        new Pose(129.000, 36.000)
                                )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            SHOOT_LAST = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.000,36.000),
                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build()
            );

            FIRST_ROW = (follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(84.000, 84.000),
                                            new Pose(128.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            SHOOT = (follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(128.000, 84.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );
            PARK = (follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(84.000, 84.000),
                                            new Pose(128.000, 93.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );
        }
    }


}
