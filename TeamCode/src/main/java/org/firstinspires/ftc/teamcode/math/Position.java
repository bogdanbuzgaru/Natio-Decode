package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Position {
    private Pose2D pose;
    private double heading;
    private final double l = 12.8740157349;     //inch
    private final double L = 14.4881889616;     //inch
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
            currentHeading -= 90;           //incadrare in patrat
        }
        return l * cos(currentHeading) + L * sin(currentHeading);
    }
    private double distanceX(){
        double currentHeading = Math.abs(heading);
        if (currentHeading > 90){
            currentHeading -= 90;           //incadrare in patrat
        }
        return L * cos(currentHeading) + l * sin(currentHeading);
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
    //-----------------------CLOSE----------------------
    private boolean isMaxXClose(){
        return (getMaxX() >= (144 - pose.getY(DistanceUnit.INCH)) && pose.getY(DistanceUnit.INCH) >= 72)
                && pose.getX(DistanceUnit.INCH) <= 72;          //The centre of the robot is on the left side of the triangle
    }
    private boolean isMinXClose(){
        return (getMinX() >= 72 && pose.getY(DistanceUnit.INCH) >= 72)          //TODO make it more up/down
                && pose.getX(DistanceUnit.INCH) >= 72;          //The centre of the robot is on the right side of the triangle
    }
    private boolean isMaxYClose(){
        return (getMaxY() >= 72 && pose.getX(DistanceUnit.INCH) >= 144 - getMaxY()      //TODO make it more to the left/right
                && pose.getX(DistanceUnit.INCH) <= getMaxY() );     //The centre of the robot is in the bottom of the triangle
    }
    //-----------------------FAR----------------------
    private boolean isMinYFar(){
        return getMinY() <= 24 && pose.getX(DistanceUnit.INCH) >= 72 - (24 - getMinY())
                && pose.getX(DistanceUnit.INCH) <= 72 + (24 - getMinY());       //Top of the far triangle
    }
    private boolean isMaxXFar(){
        return getMaxX() >= 48 && pose.getY(DistanceUnit.INCH) <= 24
                && getMaxX() - pose.getY(DistanceUnit.INCH) <= 48       //Left of the far triangle
                && pose.getY(DistanceUnit.INCH) <= 72;
    }
    private boolean isMinXFar(){
        return getMinX() <= 96 && pose.getY(DistanceUnit.INCH) <= 24
                && getMinX() + pose.getY(DistanceUnit.INCH) <= 96       //Right of the far triangle
                && pose.getY(DistanceUnit.INCH) >= 72;
    }

    public boolean shootClose(){
        return isMaxXClose() || isMaxYClose() || isMinXClose();
    }
    public boolean shootHigh(){
        return isMinYFar() || isMaxXFar() || isMinXFar();
    }
    public double getHeading(){
        return heading;
    }
    public double getTargetAngle(){
        double h = heading;
        double targetHead = Math.toDegrees(Math.atan2(144 - pose.getX(DistanceUnit.INCH), 144 - pose.getY(DistanceUnit.INCH)));
        return Math.abs(h - targetHead);
    }
}