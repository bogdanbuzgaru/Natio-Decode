package org.firstinspires.ftc.teamcode.math;

import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Position {
    private Pose pose;
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
    private boolean blue = false;
    private boolean red = true;
    private boolean shootClose = true;

    public Position (Pose pose){
        this.pose = pose;
        heading = Math.toDegrees(pose.getHeading());
        calculateOffsets();
        calculateLinearEquations();
    }

    public void update(Pose pose) {
        this.pose = pose;
        heading = Math.toDegrees(pose.getHeading());
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
        double y = pose.getY();
        return y + distanceY() / 2;
    }
    public double getMaxX(){
        double x = pose.getX();
        return x + distanceX() / 2;
    }
    public double getMinX(){
        double x = pose.getX();
        return x - distanceX() / 2;
    }
    public double getMinY(){
        double y = pose.getY();
        return y - distanceY() / 2;
    }
    private void calculateOffsets() {
        offsetAbscissa = Math.toDegrees(Math.acos((getMaxX() - pose.getX()) / semiDiagonal));
        offsetOrdinate = Math.toDegrees(Math.acos((getMaxY() - pose.getY()) / semiDiagonal));
    }
    public double getHeading(){
        return heading;
    }
    public double getAngle(){
        if(red)
            return Math.toDegrees(Math.atan2(130 - pose.getY(), 130 - pose.getX()));
        else
            return Math.toDegrees(Math.atan2(130 - pose.getY(), Math.abs(pose.getX()) - 14));   //TODO this - change +
    }


    public double getTargetAngle(){
        double targetHead = 0;
//        if(!changeCord) {
        if (red && pose.getY() >= 65)
            targetHead = Math.toDegrees(Math.atan2(Math.abs(135 - pose.getY()),Math.abs(130 - pose.getX())));

        else if (red && pose.getY() < 65 && heading < 90 && heading > -90)
            targetHead = Math.toDegrees(Math.atan2(Math.abs(138 - pose.getY()),Math.abs(133 - pose.getX())));
        else if (red && pose.getY() < 65 && heading >= 90 && heading <= -90)
            targetHead = Math.toDegrees(Math.atan2(Math.abs(138 - pose.getY()),Math.abs(160 - pose.getX())));

        else if (blue && pose.getY() >= 65){
            targetHead = 180 - Math.toDegrees(Math.atan2(Math.abs(135 - pose.getY()), Math.abs(pose.getX() - 14)));
        }
        else if (blue && pose.getY() < 65 && heading >= 90 && heading <= -90)
            targetHead = 180 - Math.toDegrees(Math.atan2(Math.abs(138 - pose.getY()),Math.abs(pose.getX() + 3)));

        else if (blue && pose.getY() < 65 && heading < 90 && heading > -90)
            targetHead = Math.toDegrees(Math.atan2(Math.abs(138 - pose.getY()),Math.abs(pose.getX() - 16)));


        double error = targetHead - heading;
        return error;
    }
    public double target(){
        double targetHead = 0;
        if(red){
            targetHead = Math.toDegrees(Math.atan2(Math.abs(136 - pose.getY()), Math.abs(136 - pose.getX())));
        }else{
            targetHead = 180 - Math.toDegrees(Math.atan2(Math.abs(136 - pose.getY()), Math.abs(pose.getX() - 8)));
        }
        double error = targetHead - heading;
        return error;
    }
    public void setBlue(){
        this.blue = true;
        this.red = false;
    }
    public void setRed(){
        this.red = true;
        this.blue = false;
    }
    public boolean isFar(){
        if(pose.getY() < 52){
            return true;
        }
        return false;
    }

    //------------------LINEAR EQUATIONS------------------
    private void calculateLinearEquations() {
        topLeft = new LinearEquation(pose.getX() + signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                 getMaxY(), getMinX(),
                 pose.getY() + (-1) * signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));

        topRight = new LinearEquation(pose.getX() + signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                getMaxY(), getMaxX(),
                pose.getY() + signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));

        bottomLeft = new LinearEquation(pose.getX() + (-1) * signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                getMinY(), getMinX(),
                pose.getY() + (-1) * signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));

        bottomRight = new LinearEquation(pose.getX() + (-1) * signOrdinate * (semiDiagonal * sin(Math.toRadians(offsetAbscissa))),
                getMinY(), getMaxX(),
                pose.getY() + signAbscissa * (semiDiagonal * sin(Math.toRadians(offsetOrdinate))));
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
                (x >= Math.min(second.getX1(), second.getX2()) && x <=  Math.max(second.getX1(), second.getX2())) &&
                (y >= Math.min(first.getY1(), first.getY2()) && y <=  Math.max(first.getY1(), first.getY2())) &&
                (y >= Math.min(second.getY1(), second.getY2()) && y <= Math.max(second.getY1(), second.getY2()));
    }

    public boolean isCenterInBigTriangle(){
        return pose.getY() >= 72 &&
                pose.getX() >= 144 - pose.getY() &&
                pose.getX() <= pose.getY();
    }
    public boolean isCenterInSmallTriangle() {
        return pose.getY() <= 24 &&
                pose.getX() >= 48 + pose.getY() &&
                pose.getX() <= 96 - pose.getY();
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
        if(red) {
            double dx = 132 - pose.getX();
            double dy = 132 - pose.getY();
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
        }else if (blue){
            double dx = Math.abs(pose.getX() - 12);
            double dy = Math.abs(132 - pose.getY());
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
        return 0;
    }
    public boolean activateOrientation(){
        double hypoHigh = Math.hypot(Math.abs(72 - pose.getX()), 144 - pose.getY());
        double hypoLow = Math.hypot(Math.abs(72 - pose.getX()), pose.getY());
        return hypoHigh <= 90 || hypoLow <= 50;
    }
    public void chooseAlliance(Gamepad gamepad){
        if(gamepad.leftBumperWasPressed()){
            blue = true;
            red = false;
        }else if (gamepad.rightBumperWasPressed()){
            red = true;
            blue = false;
        }
    }
    public int getTicks(double slope, double extra){
        double hypo = Math.hypot(Math.abs(130 - pose.getX()), Math.abs(130 - pose.getY()));
        return (int)((int) slope * hypo + extra);
    }
    public int getTicksBlue(double slope, double extra){
        double hypo = Math.hypot(Math.abs(pose.getX() - 14), Math.abs(130 - pose.getY()));
        return (int)((int) slope * hypo + extra);
    }
    public void whereToShoot(Gamepad gamepad){
        if(gamepad.dpadLeftWasPressed()){
            shootClose = true;
        }else if(gamepad.dpadRightWasPressed()){
            shootClose = false;
        }
    }
    public boolean isBlue() {
        return blue;
    }

    public boolean isRed() {
        return red;
    }
    public double offsetAngleRed(double robotVelocityX, double robotVelocityY, double vx) {
        double dx = Math.abs(130 - pose.getX());
        double dy = Math.abs(130 - pose.getY());
        double robotToGoalTheta = Math.atan2(dy, dx);

        double robotSpeed = Math.hypot(robotVelocityX, robotVelocityY);
        double robotVelocityTheta = Math.atan2(robotVelocityY, robotVelocityX);

        double coordinateTheta = robotVelocityTheta - robotToGoalTheta;
        double parallelComponent = -Math.cos(coordinateTheta) * robotSpeed;
        double perpendicularComponent =  Math.sin(coordinateTheta) * robotSpeed;

        double ivr = vx + parallelComponent;

        return Math.toDegrees(Math.atan2(perpendicularComponent, ivr));
    }
    public double offsetAngleBlue(double robotVelocityX, double robotVelocityY, double vx) {

        double dx = Math.abs(pose.getX() - 14);
        double dy = Math.abs(130 - pose.getY());
        double robotToGoalTheta = Math.atan2(dy, dx);

        double robotSpeed = Math.hypot(robotVelocityX, robotVelocityY);
        double robotVelocityTheta = Math.atan2(robotVelocityY, robotVelocityX);

        double coordinateTheta = robotVelocityTheta - robotToGoalTheta;
        double parallelComponent     = -Math.cos(coordinateTheta) * robotSpeed;
        double perpendicularComponent =  Math.sin(coordinateTheta) * robotSpeed;

        double ivr = vx + parallelComponent;

        return Math.toDegrees(Math.atan2(perpendicularComponent, ivr));
    }
}