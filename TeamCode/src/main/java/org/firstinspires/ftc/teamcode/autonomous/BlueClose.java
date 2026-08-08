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
        SHOOT_GOAL,
        TAKE_RANDOM,
        SHOOT_RANDOM,
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
    private int goalCyclesDone = 0;
    private int GOAL_CYCLES = 7;
    private ElapsedTime auto;
    private final int SHOOT_FIRST_MS = 600, SHOOT_MS = 650, WAIT_MS = 900;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(37.930, 132.1935, Math.toRadians(180)));
        paths = new Paths(follower);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake (hardwareMap);
        index = new Index(hardwareMap);
        pos = new Position(follower.getPose());
        pos.setRed();
        auto = new ElapsedTime();
//        position = new Position(new Pose2D(DistanceUnit.INCH,120.000 - 9.7322834608,
//                144.000 - 6.67322833945, AngleUnit.DEGREES, 0));
    }
    public void start(){
        setUp();
        fsm.init();
        shooter.lowerBarrier();
        auto.reset();
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
        index.normalIndex();
//        turret.setTargetAngle(pos.target()); //+ angle if needed
//        turret.setHeading(Math.toDegrees(follower.getHeading()), true);
        shooter.setTicks(1240, false, false);
        shooter.update();
        if(auto.seconds() > 59.8){
            requestOpModeStop();
        }
    }
    public void stop(){
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(Math.toDegrees(pose.getHeading()));

        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, 0 + "\n" + xPose + "\n" + yPose + "\n" + heading);

    }
    private AutoState handleShoot(AutoState nextState, long durationMs) {
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
    private AutoState gate(AutoState nextState, long durationMs){
        if (!isShooting) {
            pathTimer.reset();
            isShooting = true;
        }
        index.autoFeed();
        if (pathTimer.milliseconds() > durationMs) {
            isShooting = false;
            return nextState;
        }
        return null;
    }
    private void setUp() {
        fsm.onStateEnter(AutoState.SHOOT_FIRST, () -> {
            follower.followPath(paths.SHOOT_FIRST);
            shooter.lowerBarrier();
            turret.angleToPos(-46.2);
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_FIRST, () -> {
            turret.angleToPos(-46.2);
            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoState.SECOND_ROW, SHOOT_FIRST_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.SECOND_ROW, () -> {
            follower.followPath(paths.SECOND_ROW);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SECOND_ROW, () -> {
            turret.angleToPos(-46.2);
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
                return handleShoot(AutoState.GO_TO_GOAL, SHOOT_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.GO_TO_GOAL, () -> {
            follower.followPath(paths.GO_TO_GOAL);
            shooter.lowerBarrier();
            index.normalIndex();
            return null;
        });
        fsm.onStateUpdate(AutoState.GO_TO_GOAL, () -> {
            intake.leaveGate();
            index.autoFeed();
            if (!follower.isBusy()) {
                return gate(AutoState.SHOOT_GOAL, WAIT_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.SHOOT_GOAL, () -> {
            follower.followPath(paths.SHOOT_GOAL);
            turret.angleToPos(-43.2);
            goalCyclesDone++;
            shooter.lowerBarrier();
            index.lowerIndex();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_GOAL, () -> {
            turret.angleToPos(-43.2);
            intake.leaveGate();
            if (!follower.isBusy()) {
                if (goalCyclesDone == GOAL_CYCLES - 3) {
                    turret.angleToPos(-46.2);
                    return handleShoot(AutoState.TAKE_RANDOM, SHOOT_MS);
                } else if (goalCyclesDone <= GOAL_CYCLES) {
                    return handleShoot(AutoState.GO_TO_GOAL, SHOOT_MS);
                } else {
                    return handleShoot(AutoState.FIRST_ROW, SHOOT_MS);
                }
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
                return handleShoot(AutoState.FIRST_ROW, SHOOT_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.TAKE_RANDOM, () -> {
            follower.followPath(paths.TAKE_RANDOM);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.TAKE_RANDOM, () -> {
            intake.autoTake();
            index.autoFeed();
            if (!follower.isBusy()) {
                return AutoState.SHOOT_RANDOM;
            }
            return null;
        });
        fsm.onStateEnter(AutoState.SHOOT_RANDOM, () -> {
            follower.followPath(paths.SHOOT_RANDOM);
            turret.angleToPos(-46.2);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_RANDOM, () -> {
            intake.leaveGate();
            turret.angleToPos(-46.2);
            if (!follower.isBusy()) {
                return handleShoot(AutoState.GO_TO_GOAL, SHOOT_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.FIRST_ROW, () -> {
            follower.followPath(paths.FIRST_ROW);
            turret.angleToPos(-46.2);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.FIRST_ROW, () -> {
            turret.angleToPos(-46.2);
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
            turret.angleToPos(-46.2);
//            intake.autoTake();
            if (!follower.isBusy()) {
                return handleShoot(AutoState.PARK, SHOOT_MS);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.PARK, () -> {
            follower.followPath(paths.PARK);
            shooter.lowerBarrier();
            return null;
        });
        fsm.onStateUpdate(AutoState.PARK, () -> {
            if (!follower.isBusy()) {
                requestOpModeStop();
            }
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
        public PathChain TAKE_RANDOM;
        public PathChain SHOOT_RANDOM;

        private double actualPositionX(double value){
            return value - 9.7322834608;
        }
        private double actualPositionY(double value){
            return value - 6.692913 - 1.968504;
        } //1.968504 (intake extension)
        public Paths(Follower follower) {
            SHOOT_FIRST = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(37.930, 132.1935),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );

//            PREPARE_SECOND = (follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(54.000, 90.000),
//                                    new Pose(44.000, 67.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build()
//            );

            SECOND_ROW = (follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(44.000,67.000),
//                                    new Pose(10.000, 65.000)
//                            )
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(59.678, 51.681),
                                    new Pose(28.566, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );

            SHOOT_SECOND = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(28.566, 60.000),
                                    new Pose(29.000, 62.000),
                                    new Pose(45.000, 65.000),
                                    new Pose(50.000, 72.000),
                                    new Pose(52.000, 79.000),
                                    new Pose(53.000, 81.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );

            GO_TO_GOAL = (follower.pathBuilder().addPath(
//                                    new BezierLine(
//                                            new Pose(54.000, 90.000),
//                                            new Pose(41.000, 59.000)
//                                    )
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(49.000, 66.000),
                                    new Pose (18.300, 59.000),
                                    new Pose (17.200, 62.700),
                                    new Pose(16.500, 61.900)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(164.1))
                    .build()
            );

            SHOOT_GOAL = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.500, 61.000),
                                    new Pose(48.745, 62.783),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(164.1), Math.toRadians(180))
                    .build()
            );
//            PREPARE_LAST_ROW = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(54.000, 90.000),
//                                    new Pose(38.000, 40.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
            LAST_ROW = (follower.pathBuilder().addPath(
//                                    new BezierLine(
//                                            new Pose (38.000, 40.000),
//                                            new Pose(10.000, 40.000)
//                                    )
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(55.367, 23.549),
                                    new Pose(16.100, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );

            SHOOT_LAST = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.000, 36.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );
            TAKE_RANDOM = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(43.745, 62.000),
                                    new Pose(18.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );

            SHOOT_RANDOM = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 61.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );
            FIRST_ROW = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 84.000),
                                    new Pose(25.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );

            SHOOT = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.000, 84.000),
                                    new Pose(60.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );
            PARK = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 84.000),
                                    new Pose(32.000, 86.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build()
            );
        }
    }
}