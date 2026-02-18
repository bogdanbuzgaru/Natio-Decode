package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Position {
    private Pose2D pose;
    private double heading;
    private final double l = 12.8740157349;
    private final double L = 14.4881889616;
    public Position (Pose2D pose){
        this.pose = pose;
    }

    public void setPose(Pose2D pose) {
        this.pose = pose;
        heading = pose.getHeading(AngleUnit.DEGREES);
    }
    private double distanceY(){
        double currentHeading = Math.abs(heading);
        if (currentHeading > 90){
            currentHeading -= 90;
        }
        return 2 * l * cos(currentHeading);
    }
    private double distanceX(){
        double currentHeading = Math.abs(heading);
        if (currentHeading > 90){
            currentHeading -= 90;
        }
        return L * cos(currentHeading + l * cos(90 - currentHeading));
    }

    public double getMaxY(){
        double y = pose.getY(DistanceUnit.INCH);
        return y + distanceY() / 2;
    }
    public double getMaxX(){
        double x = pose.getX(DistanceUnit.INCH);
        return x + distanceX() / 2;
    }
    public double getMinX(){
        double x = pose.getX(DistanceUnit.INCH);
        return x - distanceX() / 2;
    }
    public double getMinY(){
        double y = pose.getY(DistanceUnit.INCH);
        return y - distanceY() / 2;
    }
}