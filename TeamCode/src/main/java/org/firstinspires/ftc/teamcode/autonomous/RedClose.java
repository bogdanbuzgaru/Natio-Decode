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
import java.util.ArrayList;

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
        public ArrayList<PathChain> paths = new ArrayList<>();

        public Paths(Follower follower) {
            paths.add( follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(112.000, 135.000),

                                    new Pose(84.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(84.000, 84.000),
                                            new Pose(86.000, 70.175),
                                            new Pose(92.000, 65.000),
                                            new Pose(105.000, 63.000),
                                            new Pose(134.000, 60.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(134.000, 60.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(84.000, 84.000),
                                            new Pose(129.000, 63.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(129.000, 63.000),
                                            new Pose(130.000, 58.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(43))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(130.000, 58.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(84.000, 84.000),
                                            new Pose(134.000, 36.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(134.000, 36.000),
                                            new Pose(84.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(84.000, 84.000),
                                            new Pose(128.000, 88.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                            .build()
            );
        }
    }


}
