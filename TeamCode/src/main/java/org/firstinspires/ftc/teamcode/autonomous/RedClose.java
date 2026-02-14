package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous
public class RedClose extends OpMode {
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
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(112.000, 135.000),
                                    new Pose(103.910, 122.032),
                                    new Pose(103.684, 113.739),
                                    new Pose(93.615, 105.169),
                                    new Pose(91.422, 94.690),
                                    new Pose(85.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 84.000),

                                    new Pose(125.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 84.000),

                                    new Pose(126.000, 75.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.000, 75.000),

                                    new Pose(85.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 84.000),

                                    new Pose(100.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 60.000),

                                    new Pose(127.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.000, 60.000),

                                    new Pose(85.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 84.000),

                                    new Pose(129.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.000, 63.000),

                                    new Pose(130.000, 57.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(43))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 57.000),

                                    new Pose(85.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 84.000),

                                    new Pose(100.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 36.000),

                                    new Pose(130.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 36.000),

                                    new Pose(85.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 84.000),

                                    new Pose(128.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }

}
