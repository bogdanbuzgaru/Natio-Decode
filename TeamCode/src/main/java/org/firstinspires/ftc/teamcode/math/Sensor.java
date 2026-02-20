package org.firstinspires.ftc.teamcode.math;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


public class Sensor {

    private final float[] hsvValues = new float[3];

    private final NormalizedColorSensor sensor;
    public static float r, g, b;
    private static final float GAIN = 25.0f;

    public Sensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, name);

        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight)sensor).enableLight(true);
        }
        sensor.setGain(GAIN);
    }

    public void updateColors() {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
    }

    // --- LOGIC ---

    public boolean isPurple() {
        boolean correctHue = (hsvValues[0] >= 210 && hsvValues[0] <= 330);
        boolean isSaturated = hsvValues[1] > 0.4;
        boolean isCloseEnough = hsvValues[2] > 0.1;

        return correctHue && isSaturated && isCloseEnough;
    }

    public boolean isGreen() {
        boolean correctHue = (hsvValues[0] >= 90 && hsvValues[0] <= 180);
        boolean isSaturated = hsvValues[1] > 0.4;
        boolean isCloseEnough = hsvValues[2] > 0.1;

        return correctHue && isSaturated && isCloseEnough;
    }

    // --- DEBUGGING HELPERS ---
    public float getHue() { return hsvValues[0]; }
    public float getSaturation() { return hsvValues[1]; }
    public float getValue() { return hsvValues[2]; }
    public double RedAmount() { return r; }
    public double GreenAmount() { return g; }
    public double BlueAmount() { return b; }
}