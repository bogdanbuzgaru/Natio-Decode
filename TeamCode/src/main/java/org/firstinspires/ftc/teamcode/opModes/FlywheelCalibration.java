package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * FLYWHEEL CALIBRATION OPMODE
 *
 * This OpMode helps you collect data to calibrate your flywheel speed.
 *
 * INSTRUCTIONS:
 * 1. Set hood to 45 degrees (measure with protractor)
 * 2. Use DPAD UP/DOWN to adjust flywheel speed
 * 3. Press A to record current speed
 * 4. Shoot ball and measure horizontal distance
 * 5. Enter distance below and it calculates launch speed
 * 6. Repeat at 6 different speeds
 * 7. Export data to Excel and create trendline
 *
 * SPEEDS TO TEST:
 * - 500 ticks/sec
 * - 700 ticks/sec
 * - 900 ticks/sec
 * - 1100 ticks/sec
 * - 1300 ticks/sec
 * - 1500 ticks/sec
 */
@TeleOp(name = "Flywheel Calibration", group = "Tuning")
public class FlywheelCalibration extends OpMode {

    private DcMotorEx flywheelMotor1, flywheelMotor2;
    private Servo hoodServo;
    private Servo barrier;

    // Current flywheel speed
    private int currentSpeed = 500;

    // Data collection
    private int[] recordedSpeeds = new int[6];
    private double[] measuredRanges = new double[6];
    private int currentDataPoint = 0;

    // Constants
    private static final double GRAVITY = 386.1; // in/s^2
    private static final int SPEED_INCREMENT = 50;
    private static final int MIN_SPEED = 0;
    private static final int MAX_SPEED = 3000;

    // State
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean recording = false;

    @Override
    public void init() {
        // Initialize hardware
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        barrier = hardwareMap.get(Servo.class, "barrier");

        // Setup motors
        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor1.setDirection(DcMotorEx.Direction.REVERSE);

        // Lower barrier
        barrier.setPosition(0.0);

        telemetry.addLine("=== FLYWHEEL CALIBRATION ===");
        telemetry.addLine();
        telemetry.addLine("SETUP:");
        telemetry.addLine("1. Set hood to 45° (measure with protractor)");
        telemetry.addLine("2. Clear 15+ foot shooting area");
        telemetry.addLine("3. Ready to shoot");
        telemetry.addLine();
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Button debouncing
        boolean dpadUpPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        boolean xPressed = gamepad1.x && !lastX;

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;

        // Adjust flywheel speed
        if (dpadUpPressed) {
            currentSpeed += SPEED_INCREMENT;
            if (currentSpeed > MAX_SPEED) currentSpeed = MAX_SPEED;
        }
        if (dpadDownPressed) {
            currentSpeed -= SPEED_INCREMENT;
            if (currentSpeed < MIN_SPEED) currentSpeed = MIN_SPEED;
        }

        // Fine adjustment with bumpers
        if (gamepad1.right_bumper) {
            currentSpeed += 10;
            if (currentSpeed > MAX_SPEED) currentSpeed = MAX_SPEED;
        }
        if (gamepad1.left_bumper) {
            currentSpeed -= 10;
            if (currentSpeed < MIN_SPEED) currentSpeed = MIN_SPEED;
        }

        // Record data point
        if (aPressed && currentDataPoint < 6) {
            recordedSpeeds[currentDataPoint] = currentSpeed;
            recording = true;
            telemetry.clear();
            telemetry.addLine(">>> DATA POINT RECORDED <<<");
            telemetry.addData("Speed", "%d ticks/sec", currentSpeed);
            telemetry.addLine();
            telemetry.addLine("NOW:");
            telemetry.addLine("1. SHOOT the ball");
            telemetry.addLine("2. MEASURE horizontal distance (inches)");
            telemetry.addLine("3. REMEMBER the distance");
            telemetry.addLine("4. Press X when ready to enter distance");
            telemetry.update();
        }

        // Enter measured range (simplified - just advances to next)
        if (xPressed && recording) {
            // In a real scenario, you'd have a way to input the measured range
            // For now, we'll just show instructions
            telemetry.clear();
            telemetry.addLine(">>> ENTER MEASURED RANGE <<<");
            telemetry.addLine();
            telemetry.addLine("WRITE DOWN:");
            telemetry.addData("Test #", "%d", currentDataPoint + 1);
            telemetry.addData("Flywheel Speed", "%d ticks/sec", currentSpeed);
            telemetry.addLine("Measured Range: _____ inches");
            telemetry.addLine();
            telemetry.addLine("Then press B to continue");
            telemetry.update();
        }

        // Continue to next data point
        if (bPressed && recording) {
            recording = false;
            currentDataPoint++;

            if (currentDataPoint < 6) {
                // Suggest next speed to test
                int[] suggestedSpeeds = {500, 700, 900, 1100, 1300, 1500};
                currentSpeed = suggestedSpeeds[currentDataPoint];
            }
        }

        // Spin flywheels when holding right trigger
        if (gamepad1.right_trigger > 0.1) {
            flywheelMotor1.setVelocity(currentSpeed);
            flywheelMotor2.setVelocity(currentSpeed);
            barrier.setPosition(1.0); // Raise barrier to shoot
        } else {
            flywheelMotor1.setVelocity(0);
            flywheelMotor2.setVelocity(0);
            barrier.setPosition(0.0); // Lower barrier
        }

        // Display main screen when not recording
        if (!recording) {
            telemetry.clear();
            telemetry.addLine("=== FLYWHEEL CALIBRATION ===");
            telemetry.addLine();

            // Current settings
            telemetry.addData("Flywheel Speed", "%d ticks/sec", currentSpeed);
            telemetry.addData("Progress", "%d / 6 tests", currentDataPoint);
            telemetry.addLine();

            // Controls
            telemetry.addLine("CONTROLS:");
            telemetry.addLine("DPAD UP/DOWN: Adjust speed (±50)");
            telemetry.addLine("BUMPERS: Fine adjust (±10)");
            telemetry.addLine("HOLD RT: Spin flywheels & shoot");
            telemetry.addLine("A: Record this speed");
            telemetry.addLine();

            // Suggested speeds
            telemetry.addLine("SUGGESTED SPEEDS:");
            telemetry.addLine("500, 700, 900, 1100, 1300, 1500");
            telemetry.addLine();

            // Data collected so far
            if (currentDataPoint > 0) {
                telemetry.addLine("=== DATA COLLECTED ===");
                for (int i = 0; i < currentDataPoint; i++) {
                    telemetry.addData("Test " + (i + 1), "%d ticks/sec", recordedSpeeds[i]);
                }
                telemetry.addLine();
            }

            // Instructions
            if (currentDataPoint < 6) {
                telemetry.addLine("READY TO SHOOT:");
                telemetry.addLine("1. Set speed with DPAD");
                telemetry.addLine("2. Press A to record");
                telemetry.addLine("3. Hold RT to shoot");
                telemetry.addLine("4. Measure distance");
                telemetry.addLine("5. Write down the data");
            } else {
                telemetry.addLine(">>> ALL DATA COLLECTED! <<<");
                telemetry.addLine();
                telemetry.addLine("=== RECORDED SPEEDS ===");
                for (int i = 0; i < 6; i++) {
                    telemetry.addData("Test " + (i + 1), "%d ticks/sec", recordedSpeeds[i]);
                }
                telemetry.addLine();
                telemetry.addLine("=== NEXT STEPS ===");
                telemetry.addLine("For each test, calculate:");
                telemetry.addLine("Launch Speed = sqrt(Range × 386.1)");
                telemetry.addLine();
                telemetry.addLine("Example:");
                telemetry.addLine("Range = 120 inches");
                telemetry.addLine("Launch Speed = sqrt(120 × 386.1)");
                telemetry.addLine("             = 215.3 in/s");
                telemetry.addLine();
                telemetry.addLine("Then in Excel:");
                telemetry.addLine("Column A: Flywheel Speed (ticks/sec)");
                telemetry.addLine("Column B: Launch Speed (in/s)");
                telemetry.addLine("Create scatter plot");
                telemetry.addLine("Add linear trendline");
                telemetry.addLine("Display equation: y = mx + b");
                telemetry.addLine("Use m and b in your code");
            }

            telemetry.update();
        }
    }

    @Override
    public void stop() {
        // Stop flywheels
        flywheelMotor1.setVelocity(0);
        flywheelMotor2.setVelocity(0);
        barrier.setPosition(0.0);
    }
}