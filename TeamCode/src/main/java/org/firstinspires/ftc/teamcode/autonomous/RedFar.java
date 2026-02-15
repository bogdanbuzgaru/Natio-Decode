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
        SHOOT,
        DETECT,
        COLECT;
    }
    private StateMachine<AutoStates> fsm = new StateMachine<>(AutoStates.SHOOT);

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
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Park;


        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.000, 8.000),

                                    new Pose(134.000, 9.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 9.000),

                                    new Pose(94.000, 8.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
            Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(94.000, 8.500),
                                new Pose (111.000, 16.000)
                        )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}
