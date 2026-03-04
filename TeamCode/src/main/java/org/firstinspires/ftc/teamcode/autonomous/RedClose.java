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
import java.util.ArrayList;

@Autonomous
public class RedClose extends OpMode {

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

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(120.000 - 9.7322834608, 144.000 - 6.67322833945));
        new Paths(follower);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake (hardwareMap);
        index = new Index(hardwareMap);
        position = new Position(new Pose2D(DistanceUnit.INCH,120.000 - 9.7322834608, 144.000 - 6.67322833945, AngleUnit.DEGREES, 0  ));
        setUp();
        fsm.init();
    }
    public void loop(){
        follower.update();
        position.update(new Pose2D(DistanceUnit.INCH,
                follower.getPose().getX(),
                follower.getPose().getY(),
                AngleUnit.DEGREES,
                follower.getHeading()));
        turret.update();
        shooter.setTicks(position.getTicks(8.8057, 1098));
        shooter.update();
        fsm.update();
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
    private void setUp(){
        fsm.onStateEnter(AutoState.SHOOT_FIRST, () -> {
            follower.followPath(Paths.paths.get(0));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.SHOOT_FIRST, () -> {
            intake.autoTake();
            if(!follower.isBusy()) {
                return handleShoot(AutoState.SECOND_ROW, 700);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.SECOND_ROW, () -> {
            follower.followPath(Paths.paths.get(1));
            shooter.lowerBarrier();
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
            follower.followPath(Paths.paths.get(2));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.SHOOT_SECOND, () -> {
            intake.autoTake();
            if(!follower.isBusy()) {
                return handleShoot(AutoState.GO_TO_GOAL, 700);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.GO_TO_GOAL, () -> {
            follower.followPath(Paths.paths.get(3));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.GO_TO_GOAL, () -> {
            intake.autoTake();
            index.autoFeed();
            if(!follower.isBusy()) {
                return AutoState.GO_BACK;
            }
            return null;
        });
        fsm.onStateEnter(AutoState.GO_BACK, () -> {
            follower.followPath(Paths.paths.get(4));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.GO_BACK, () -> {
            intake.autoTake();
            index.autoFeed();
            if(!follower.isBusy()) {
                return AutoState.SHOOT_THIRD;
            }
            return null;
        });
        fsm.onStateEnter(AutoState.SHOOT_THIRD, () -> {
            follower.followPath(Paths.paths.get(5));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.SHOOT_THIRD, () -> {
            intake.autoTake();
            if(!follower.isBusy()) {
                return handleShoot(AutoState.LAST_ROW, 700);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.LAST_ROW, () -> {
            follower.followPath(Paths.paths.get(6));
            shooter.lowerBarrier();
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
            follower.followPath(Paths.paths.get(7));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.SHOOT_LAST, () -> {
            intake.autoTake();
            if(!follower.isBusy()) {
                return handleShoot(AutoState.FIRST_ROW, 700);

            }
            return null;
        });
        fsm.onStateEnter(AutoState.FIRST_ROW, () -> {
            follower.followPath(Paths.paths.get(8));
            shooter.lowerBarrier();
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
            follower.followPath(Paths.paths.get(9));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.SHOOT, () -> {
            intake.autoTake();
            if(!follower.isBusy()) {
                return handleShoot(AutoState.PARK, 700);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.PARK, () -> {
            follower.followPath(Paths.paths.get(10));
            shooter.lowerBarrier();
        });
        fsm.onStateUpdate(AutoState.PARK, () -> {
            intake.autoTake();
            if(!follower.isBusy()) {
                requestOpModeStop();
            }
            return null;
        });


    }
    public static class Paths {
        public static ArrayList<PathChain> paths = new ArrayList<>();
        private double actualPositionX(double value){
            return value - 9.7322834608;
        }
        private double actualPositionY(double value){
            return value - 6.67322833945;
        }
        public Paths(Follower follower) {
            paths.clear();
            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(actualPositionX(120.000), actualPositionY(144.000)),
                                            new Pose(110.000, 130.000),
                                            new Pose(95.000, 105.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(84.000, 84.000),
                                            new Pose(89.000, 74.000),
                                            new Pose(97.000, 66.000),
                                            new Pose(112.000, 62.000),
                                            new Pose(127.000, 60.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(127.000, 60.000),
                                            new Pose(115.000, 60.000),
                                            new Pose(99.000, 68.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(84.000, 84.000),
                                            new Pose(91.000, 76.000),
                                            new Pose(104.000, 67.000),
                                            new Pose(120.000, 63.000),
                                            new Pose(129.000, 63.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(129.000, 63.000),
                                            new Pose(129.500, 61.000),
                                            new Pose(130.000, 58.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(43))
                            .build()
            );


            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(130.000, 58.000),
                                            new Pose(130.000, 65.000),
                                            new Pose(115.000, 72.000),
                                            new Pose(97.000, 78.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(84.000, 84.000),
                                            new Pose(88.000, 68.000),
                                            new Pose(93.000, 54.000),
                                            new Pose(108.000, 40.000),
                                            new Pose(134.000, 36.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(134.000, 36.000),
                                            new Pose(120.000, 38.000),
                                            new Pose(105.000, 48.000),
                                            new Pose(94.000, 64.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(84.000, 84.000),
                                            new Pose(100.000, 86.000),
                                            new Pose(114.000, 85.000),
                                            new Pose(128.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(128.000, 84.000),
                                            new Pose(114.000, 82.000),
                                            new Pose(100.000, 83.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );
            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(84.000, 84.000),
                                            new Pose(100.000, 87.000),
                                            new Pose(112.000, 89.000),
                                            new Pose(120.000, 89.000),
                                            new Pose(128.000, 88.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );
        }
    }


}
