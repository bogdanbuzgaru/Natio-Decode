package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.cos;
import static java.lang.Math.hypot;
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
    private final double GRAVITY = 386.1;       //inch / s^2
    private final double goalHeight = 45;     //inch
    private final double goalEntryAngle = Math.toRadians(-30);
    private final double semiDiagonal = 9.69081911566; //inch
    private final double inRobotAngle = 83.2477481 ; //degrees
    private final LinearEquation leftBigTriangle = new LinearEquation(14, 130, 72, 72);
    private final LinearEquation rightBigTriangle = new LinearEquation(130, 130, 72, 72);
    private final LinearEquation leftSmallTriangle = new LinearEquation(48, 0, 72, 24);
    private final LinearEquation rightSmallTriangle = new LinearEquation(72, 24, 96, 0);
    private LinearEquation topLeft = new LinearEquation( 0, 0, 0, 0),
            topRight = new LinearEquation(0, 0, 0, 0),
            bottomLeft = new LinearEquation(0, 0, 0, 0),
            bottomRight = new LinearEquation(0, 0, 0, 0);
    private double offsetAbscissa, offsetOrdinate;

    public Position (Pose2D pose){
        this.pose = pose;
        heading = pose.getHeading(AngleUnit.DEGREES);
        calculateOffsets();
        calculateLinearEquations();
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
            currentHeading = 180 - currentHeading;           //incadrare in patrat
        }
        return l * cos(Math.toRadians(currentHeading)) + L * sin(Math.toRadians(currentHeading));
    }
    private double distanceX(){
        double currentHeading = Math.abs(heading);
        if (currentHeading > 90){
            currentHeading = 180 - currentHeading;           //incadrare in patrat
        }
        return L * cos(Math.toRadians(currentHeading)) + l * sin(Math.toRadians(currentHeading));
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
        offsetAbscissa = Math.toDegrees(Math.acos((getMaxX() - pose.getX(DistanceUnit.INCH)) / semiDiagonal));
        offsetOrdinate = Math.toDegrees(Math.acos((getMaxY() - pose.getY(DistanceUnit.INCH)) / semiDiagonal));
    }
    public double getHeading(){
        return heading;
    }
    public double getAngle(){
        return Math.toDegrees(Math.atan2(144 - pose.getY(DistanceUnit.INCH), 144 - pose.getX(DistanceUnit.INCH)));
    }
    public double getTargetAngle(){
        double h = heading;
        double targetHead = Math.toDegrees(Math.atan2(144 - pose.getY(DistanceUnit.INCH), 144 - pose.getX(DistanceUnit.INCH)));
        if(Math.abs(h - targetHead) > 180){
            return 360 - Math.abs(h - targetHead);
        }
        return Math.abs(h - targetHead);
    }
    //------------------LINEAR EQUATIONS------------------
    private void calculateLinearEquations() {
        topLeft = new LinearEquation(pose.getX(DistanceUnit.INCH) + signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                 getMaxY(), getMinX(),
                 pose.getY(DistanceUnit.INCH) + (-1) * signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));

        topRight = new LinearEquation(pose.getX(DistanceUnit.INCH) + signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                getMaxY(), getMaxX(),
                pose.getY(DistanceUnit.INCH) + signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));

        bottomLeft = new LinearEquation(pose.getX(DistanceUnit.INCH) + (-1) * signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                getMinY(), getMinX(),
                pose.getY(DistanceUnit.INCH) + (-1) * signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));

        bottomRight = new LinearEquation(pose.getX(DistanceUnit.INCH) + (-1) * signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                getMinY(), getMaxX(),
                pose.getY(DistanceUnit.INCH) + signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));
    }
    public boolean areIntersecting(LinearEquation first, LinearEquation second) {
        if(Math.abs(first.getSlope() - second.getSlope()) < 1e-9){
            return false;
        }
        double a = first.getxCoeff(), b = first.getyCoeff(), c = first.getConstant();
        double alpha = second.getxCoeff(), beta = second.getyCoeff(), gamma = second.getConstant();
        double x = (b * gamma - c * beta) / (a * beta - b * alpha);
        double y = (c * alpha - a * gamma) / (a * beta - b * alpha);
        return (x >= Math.min(first.getX1(), first.getX2()) && x <= Math.max(first.getX1(), first.getX2())) &&
                (x >= Math.min(second.getX1(), second.getX2()) && x <= Math.max(second.getX1(), second.getX2())) &&
                (y >= Math.min(first.getY1(), first.getY2()) && y <= Math.max(first.getY1(), first.getY2())) &&
                (y >= Math.min(second.getY1(), second.getY2()) && y <= Math.max(second.getY1(), second.getY2()));
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
    private boolean isTangentToBigTriangle(){
        return areIntersecting(topLeft, leftBigTriangle) || areIntersecting(topLeft, rightBigTriangle) ||
                areIntersecting(topRight, leftBigTriangle) || areIntersecting(topRight, rightBigTriangle) ||
                areIntersecting(bottomLeft, leftBigTriangle) || areIntersecting(bottomLeft, rightBigTriangle) ||
                areIntersecting(bottomRight, leftBigTriangle) || areIntersecting(bottomRight, rightBigTriangle);
    }
    private boolean isTangentToSmallTriangle(){
        return areIntersecting(bottomLeft, leftSmallTriangle) || areIntersecting(bottomLeft, rightSmallTriangle) ||
                areIntersecting(bottomRight, leftSmallTriangle) || areIntersecting(bottomRight, rightSmallTriangle) ||
                areIntersecting(topLeft, leftSmallTriangle) || areIntersecting(topLeft, rightSmallTriangle) ||
                areIntersecting(topRight, leftSmallTriangle) || areIntersecting(topRight, rightSmallTriangle);
    }
    public boolean shootClose(){
        return isCenterInBigTriangle() || isTangentToBigTriangle();
    }
    public boolean shootHigh(){
        return isCenterInSmallTriangle() || isTangentToSmallTriangle();
    }
    public double getOffetAngle(double robotVelocityX, double robotVelocityY){
        double dx = 138 - pose.getX(DistanceUnit.INCH);
        double dy = 138 - pose.getY(DistanceUnit.INCH);
        double distanceToGoal = hypot(dy, dx);

        double alpha = Math.atan((2.0 * goalHeight / distanceToGoal) - Math.tan(goalEntryAngle));
        double cosAlpha = Math.cos(alpha);
        double tanAlpha = Math.tan(alpha);

        double numerator = GRAVITY * distanceToGoal * distanceToGoal;
        double denominator = 2.0 * cosAlpha * cosAlpha * (distanceToGoal * tanAlpha - goalHeight);
        double v0 = Math.sqrt(numerator / denominator);
        double vx = v0 * Math.cos(alpha);

        double angleToGoal = Math.atan2(dy, dx);
        double robotSpeed = Math.sqrt(robotVelocityX * robotVelocityX + robotVelocityY * robotVelocityY);
        double robotVelocityAngle = Math.atan2(robotVelocityY, robotVelocityX);

        double deltaAngle = robotVelocityAngle - angleToGoal;
        double vRadial = -Math.cos(deltaAngle) * robotSpeed;
        double vTangential = Math.sin(deltaAngle) * robotSpeed;

        double vxCompensatedRadial = vx + vRadial;

        double turretOffsetAngle = Math.toDegrees(Math.atan2(vTangential, vxCompensatedRadial));

        return turretOffsetAngle;
    }
    public boolean activateOrientation(){
        double hypoHigh = Math.hypot(Math.abs(72 - pose.getX(DistanceUnit.INCH)), 144 - pose.getY(DistanceUnit.INCH));
        double hypoLow = Math.hypot(Math.abs(72 - pose.getX(DistanceUnit.INCH)), pose.getY(DistanceUnit.INCH));
        return hypoHigh <= 86 || hypoLow <= 40;
    }
}