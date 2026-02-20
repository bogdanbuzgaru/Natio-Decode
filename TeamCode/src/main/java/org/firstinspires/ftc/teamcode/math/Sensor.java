package org.firstinspires.ftc.teamcode.math;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


public class Sensor {

    // Store HSV values: [0]=Hue, [1]=Saturation, [2]=Value
    private final float[] hsvValues = new float[3];

    private final NormalizedColorSensor sensor;
    public static float r, g, b;
    // Adjustable Gain: Higher = more sensitive to dark objects/longer distance
    // Standard Rev Color V3 sensor works well with gain 15-30 for game elements
    private static final float GAIN = 25.0f;

    public Sensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, name);

        // 1. Turn on the white LED so we can see the ball in dark areas
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight)sensor).enableLight(true);
        }

        // 2. Set Gain to boost the signal
        sensor.setGain(GAIN);
    }

    public void updateColors() {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        // Convert the normalized RGB (0.0 to 1.0) to HSV
        Color.colorToHSV(colors.toColor(), hsvValues);
    }

    // --- LOGIC ---

    public boolean isPurple() {
        // Purple is typically around 240-300 on the Hue chart
        // We also check 'Value' (brightness) to ensure a ball is actually there
        boolean correctHue = (hsvValues[0] >= 210 && hsvValues[0] <= 330);
        boolean isSaturated = hsvValues[1] > 0.4; // Not grey/white
        boolean isCloseEnough = hsvValues[2] > 0.1; // Not looking at a black void

        return correctHue && isSaturated && isCloseEnough;
    }

    public boolean isGreen() {
        // Green is typically around 90-150 on the Hue chart
        boolean correctHue = (hsvValues[0] >= 90 && hsvValues[0] <= 180);
        boolean isSaturated = hsvValues[1] > 0.4;
        boolean isCloseEnough = hsvValues[2] > 0.1;

        return correctHue && isSaturated && isCloseEnough;
    }

    // --- DEBUGGING HELPERS ---
    // Use these to calibrate!
    public float getHue() { return hsvValues[0]; }
    public float getSaturation() { return hsvValues[1]; }
    public float getValue() { return hsvValues[2]; }
    public double RedAmount() { return r; }
    public double GreenAmount() { return g; }
    public double BlueAmount() { return b; }
}