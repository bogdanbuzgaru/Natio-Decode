package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Position {
    private Pose2D pose;
    private double heading;
    private final double l = 13.2;  //tune
    private final double L = 11.4;  //tune
    public Position (Pose2D pose){
        this.pose = pose;
    }

    public void setPose(Pose2D pose) {
        this.pose = pose;
        heading = pose.getHeading(AngleUnit.DEGREES);
    }
    public Pose2D getMaxY(){
        double y = pose.getY(DistanceUnit.INCH);
        return new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH),
                y * Math.abs((cos(Math.abs(heading) - 45) * l)),
                AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES));
    }
    public Pose2D getMaxX(){
        double x = pose.getX(DistanceUnit.INCH);
        return new Pose2D(DistanceUnit.INCH, x * Math.abs((cos(Math.abs(heading) - 45) * L)),
                pose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES));
    }
    public Pose2D getMinX(){
        double x = pose.getX(DistanceUnit.INCH);
        return new Pose2D(DistanceUnit.INCH, x * (-1) * Math.abs((cos(Math.abs(heading) - 45) * L)),
                pose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES));
    }
    public Pose2D getMinY(){
        double y = pose.getY(DistanceUnit.INCH);
        return new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH),
                y * (-1) * Math.abs((cos(Math.abs(heading) - 45) * l)),
                AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES));
    }
}
