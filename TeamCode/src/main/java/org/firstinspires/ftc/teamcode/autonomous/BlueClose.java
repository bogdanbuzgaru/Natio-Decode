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
public class BlueClose extends OpMode {
    public void init() {

    }
    public void loop() {

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
    public static class Paths {
        public ArrayList<PathChain> paths = new ArrayList<>();

        public Paths(Follower follower) {
            paths.add(
                follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(32.000, 135.000),
                        new Pose(40.090, 122.032),
                        new Pose(40.316, 113.739),
                        new Pose(50.385, 105.169),
                        new Pose(52.578, 94.690),
                        new Pose(59.000, 84.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(59.000, 84.000),
                        new Pose(19.000, 84.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(19.000, 84.000),
                        new Pose(18.000, 75.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(18.000, 75.000),
                        new Pose(59.000, 84.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(59.000, 84.000),
                        new Pose(44.000, 60.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(44.000, 60.000),
                        new Pose(17.000, 60.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180),
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(17.000, 60.000),
                        new Pose(59.000, 84.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(59.000, 84.000),
                        new Pose(15.000, 63.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(137)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(15.000, 63.000),
                        new Pose(14.000, 57.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(137), 
                    Math.toRadians(137)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(14.000, 57.000),
                        new Pose(59.000, 84.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(137), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(59.000, 84.000),
                        new Pose(44.000, 36.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(44.000, 36.000),
                        new Pose(14.000, 36.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(14.000, 36.000),
                        new Pose(59.000, 84.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180),
                    Math.toRadians(180)
                )
                .build()
            );

            paths.add(
                follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(59.000, 84.000),
                        new Pose(16.000, 88.000)
                    )
                ).setLinearHeadingInterpolation(
                    Math.toRadians(180), 
                    Math.toRadians(180)
                )
                .build()
            );
        }
    }
}
