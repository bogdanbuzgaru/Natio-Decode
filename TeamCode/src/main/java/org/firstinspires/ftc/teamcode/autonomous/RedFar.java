package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.io.File;

@Autonomous
public class RedFar extends OpMode {

    private enum AutoStates{
        PREPARE,
        TAKE_HUMAN,
        GO_SHOOT_HU,
        CENTER_LAST_ROW,
        TAKE_LAST_ROW,
        GO_SHOOT_LAST_ROW,
        PARK;
    }
    private StateMachine<AutoStates> fsm = new StateMachine<>(AutoStates.PREPARE);

    public void init(){

    }
    public void loop(){

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
    private void setUp(){
        fsm.onStateEnter(AutoStates.PREPARE, () -> {

        });
        fsm.onStateUpdate(AutoStates.PREPARE, () -> {

        });
        fsm.onStateEnter(AutoStates.TAKE_HUMAN, () -> {

        });
        fsm.onStateUpdate(AutoStates.TAKE_HUMAN, () -> {

        });
        fsm.onStateEnter(AutoStates.GO_SHOOT_HU, () -> {

        });
        fsm.onStateUpdate(AutoStates.GO_SHOOT_HU, () -> {

        });
        fsm.onStateEnter(AutoStates.CENTER_LAST_ROW, () -> {

        });
        fsm.onStateUpdate(AutoStates.CENTER_LAST_ROW, () -> {

        });
        fsm.onStateEnter(AutoStates.TAKE_LAST_ROW, () -> {

        });
        fsm.onStateUpdate(AutoStates.TAKE_LAST_ROW, () -> {

        });
        fsm.onStateEnter(AutoStates.GO_SHOOT_LAST_ROW, () -> {

        });
        fsm.onStateUpdate(AutoStates.GO_SHOOT_LAST_ROW, () -> {

        });
        fsm.onStateEnter(AutoStates.PARK, () -> {

        });
        fsm.onStateUpdate(AutoStates.PARK, () -> {

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
