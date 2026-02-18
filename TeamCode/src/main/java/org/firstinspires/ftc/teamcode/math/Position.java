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
    private double getMaxY(){
        double y = pose.getY(DistanceUnit.INCH);
        return y + distanceY() / 2;
    }
    private double getMaxX(){
        double x = pose.getX(DistanceUnit.INCH);
        return x + distanceX() / 2;
    }
    private double getMinX(){
        double x = pose.getX(DistanceUnit.INCH);
        return x - distanceX() / 2;
    }
    private double getMinY(){
        double y = pose.getY(DistanceUnit.INCH);
        return y - distanceY() / 2;
    }
    private boolean isMaxXClose(){
        return (getMaxX() >= (144 - pose.getY(DistanceUnit.INCH)) && pose.getY(DistanceUnit.INCH) >= 72)
                && pose.getX(DistanceUnit.INCH) <= 72;          //The centre of the robot is on the left side of the triangle
    }
    private boolean isMinXClose(){
        return (getMinX() >= 72 && pose.getY(DistanceUnit.INCH) >= 72)
                && pose.getX(DistanceUnit.INCH) >= 72;          //The centre of the robot is on the right side of the triangle
    }
    private boolean isMaxYClose(){
        return (getMaxY() >= 72 && pose.getX(DistanceUnit.INCH) >= 144 - getMaxY()
                && pose.getX(DistanceUnit.INCH) <= getMaxY() );     //The centre of the robot is in the bottom of the triangle
    }
    public boolean shootClose(){
        return isMaxXClose() || isMaxYClose() || isMinXClose();
    }
    public boolean shootHigh(){
        return false;
    }
}