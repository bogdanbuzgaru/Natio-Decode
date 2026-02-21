package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Position {
    private Pose2D pose;
    private double heading;
    private int signOrdinate = 1, signAbscissa = 1;
    private final double l = 12.8740157349;     //inch
    private final double L = 14.4881889616;     //inch
    private final double semiDiagonal = 9.69081911566; //inch
    private final double inRobotAngle = 83.2477481 ; //degrees
    private final LinearEquation leftBigTriangle = new LinearEquation(14, 130, 72, 72);
    private final LinearEquation rightBigTriangle = new LinearEquation(130, 130, 72, 72);
    private final LinearEquation leftSmallTriangle = new LinearEquation(48, 0, 72, 24);
    private final LinearEquation rightSmallTriangle = new LinearEquation(72, 24, 96, 0);
    private LinearEquation topLeft, topRight, bottomLeft, bottomRight;
    private double offsetAbscissa, offsetOrdinate;

    public Position (Pose2D pose){
        this.pose = pose;
    }

    public void update(Pose2D pose) {
        this.pose = pose;
        heading = pose.getHeading(AngleUnit.DEGREES);
        calculateOffsets();
        if(Math.abs(heading) > 90){
            signOrdinate = - 1;
        } else {
            signOrdinate = 1;
        }
        if(Math.abs(heading) < inRobotAngle / 2){
            signAbscissa = - 1;
        } else {
            signAbscissa = 1;
        }
        calculateLinearEquations();
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
    private void calculateOffsets() {
        offsetAbscissa = Math.acos(getMaxX() / semiDiagonal);
        offsetOrdinate = Math.acos(getMaxY() / semiDiagonal);
    }
    //------------------LINEAR EQUATIONS------------------
    private void calculateLinearEquations() {
        topLeft = new LinearEquation(pose.getX(DistanceUnit.INCH) + signOrdinate * (semiDiagonal / sin(offsetAbscissa)),
                 getMaxY(), getMinX(),
                 pose.getY(DistanceUnit.INCH) + (-1) * signAbscissa * (semiDiagonal / sin(offsetOrdinate)));

        topRight = new LinearEquation(pose.getX(DistanceUnit.INCH) + signOrdinate * (semiDiagonal / sin(offsetAbscissa)),
                getMaxY(), getMaxX(),
                pose.getY(DistanceUnit.INCH) + signAbscissa * (semiDiagonal / sin(offsetOrdinate)));

        bottomLeft = new LinearEquation(pose.getX(DistanceUnit.INCH) + (-1) * signOrdinate * (semiDiagonal / sin(offsetAbscissa)),
                getMinY(), getMinX(),
                pose.getY(DistanceUnit.INCH) + (-1) * signAbscissa * (semiDiagonal / sin(offsetOrdinate)));

        bottomRight = new LinearEquation(pose.getX(DistanceUnit.INCH) + (-1) * signOrdinate * (semiDiagonal / sin(offsetAbscissa)),
                getMinY(), getMaxX(),
                pose.getY(DistanceUnit.INCH) + signAbscissa * (semiDiagonal / sin(offsetOrdinate)));
    }
    public boolean isCenterInBigTriangle(){
        return pose.getY(DistanceUnit.INCH) >= 72 &&
                pose.getX(DistanceUnit.INCH) >= 144 - pose.getY(DistanceUnit.INCH) &&
                pose.getX(DistanceUnit.INCH) <= pose.getY(DistanceUnit.INCH);
    }
    public boolean isCenterInSmallTriangle() {
        return pose.getY(DistanceUnit.INCH) <= 24 &&
                pose.getX(DistanceUnit.INCH) >= 48 + pose.getY(DistanceUnit.INCH) &&
                pose.getX(DistanceUnit.INCH) <= 96 - pose.getY(DistanceUnit.INCH);
    }
    public boolean shootClose(){
        return isCenterInBigTriangle();
    }
    public boolean shootHigh(){
        return isCenterInSmallTriangle();
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