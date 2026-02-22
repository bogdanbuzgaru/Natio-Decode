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
        private double actualPositionX(double value){
            return value - 7.2440944808;
        }
        private double actualPositionY(double value){
            return value - 6.43700786745;
        }
        public Paths(Follower follower) {
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
