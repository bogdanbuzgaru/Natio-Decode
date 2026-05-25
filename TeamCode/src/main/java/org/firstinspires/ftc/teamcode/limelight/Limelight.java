package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {

    private Limelight3A limelight;

    private boolean targetDetected;
    private double xOffset;
    private double yOffset;
    private double targetArea;

    public Limelight(HardwareMap hardwareMap, String deviceName, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);

        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            targetDetected = true;
            xOffset = result.getTx();
            yOffset = result.getTy();
            targetArea = result.getTa();
        } else {
            targetDetected = false;
            xOffset = 0.0;
            yOffset = 0.0;
            targetArea = 0.0;
        }
    }
    public boolean hasTarget() {
        return targetDetected;
    }
    public double getXOffset() {
        return xOffset;
    }

    public double getYOffset() {
        return yOffset;
    }

    public double getTargetArea() {
        return targetArea;
    }
}
