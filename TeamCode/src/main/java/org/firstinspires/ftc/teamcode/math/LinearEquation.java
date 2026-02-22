package org.firstinspires.ftc.teamcode.math;

public class LinearEquation {
    private double slope;
    private double xCoeff, yCoeff, constant;
    private final double x1, y1, x2, y2;

    public LinearEquation(double x1, double y1, double x2, double y2) {
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        calculate();
    }
    private void calculate() {
        xCoeff = y1 - y2;
        yCoeff = x2 - x1;
        constant = x1 * y2 - x2 * y1;
        slope = -xCoeff / yCoeff;
    }

    public double getX1() {
        return x1;
    }

    public double getY1() {
        return y1;
    }

    public double getX2() {
        return x2;
    }

    public double getY2() {
        return y2;
    }

    public double getSlope() {
        return slope;
    }

    public double getxCoeff() {
        return xCoeff;
    }

    public double getyCoeff() {
        return yCoeff;
    }

    public double getConstant() {
        return constant;
    }
}
