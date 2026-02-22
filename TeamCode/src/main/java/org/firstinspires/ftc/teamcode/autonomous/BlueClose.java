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
public class BlueClose extends OpMode {
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
        private double actualPositionX(double value){
            return value + 7.2440944808;
        }
        private double actualPositionY(double value){
            return value - 6.43700786745;
        }
        public Paths(Follower follower) {
            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(actualPositionX(24.000), actualPositionY(144.000)),
                                            new Pose(34.000, 130.000),
                                            new Pose(49.000, 105.000),
                                            new Pose(60.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(60.000, 84.000),
                                            new Pose(55.000, 74.000),
                                            new Pose(47.000, 66.000),
                                            new Pose(32.000, 62.000),
                                            new Pose(17.000, 60.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(17.000, 60.000),
                                            new Pose(29.000, 60.000),
                                            new Pose(45.000, 68.000),
                                            new Pose(60.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(60.000, 84.000),
                                            new Pose(53.000, 76.000),
                                            new Pose(40.000, 67.000),
                                            new Pose(24.000, 63.000),
                                            new Pose(15.000, 63.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(15.000, 63.000),
                                            new Pose(14.500, 61.000),
                                            new Pose(14.000, 58.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(137))
                            .build()
            );


            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(14.000, 58.000),
                                            new Pose(14.000, 65.000),
                                            new Pose(29.000, 72.000),
                                            new Pose(47.000, 78.000),
                                            new Pose(60.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(60.000, 84.000),
                                            new Pose(56.000, 68.000),
                                            new Pose(51.000, 54.000),
                                            new Pose(36.000, 40.000),
                                            new Pose(10.000, 36.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(10.000, 36.000),
                                            new Pose(24.000, 38.000),
                                            new Pose(39.000, 48.000),
                                            new Pose(50.000, 64.000),
                                            new Pose(60.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(60.000, 84.000),
                                            new Pose(44.000, 86.000),
                                            new Pose(30.000, 85.000),
                                            new Pose(16.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(16.000, 84.000),
                                            new Pose(30.000, 82.000),
                                            new Pose(44.000, 83.000),
                                            new Pose(60.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );

            paths.add(follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(60.000, 84.000),
                                            new Pose(44.000, 87.000),
                                            new Pose(32.000, 89.000),
                                            new Pose(24.000, 89.000),
                                            new Pose(16.000, 88.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                            .build()
            );
        }
    }
}
