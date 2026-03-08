package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 3-servo turret using a LUT (lookup table) for angle→position mapping
 * with exponential smoothing for jitter-free motion.
 *
 * Position servos have their own internal controller, so the correct
 * approach is to compute the desired absolute position from the LUT and
 * set it directly — no incremental PD stepping needed (that pattern is
 * for DC motors, not servos, and causes severe tracking lag).
 *
 * Tuning guide:
 *   SMOOTHING_ALPHA         – 0 = frozen, 1 = instant jump; 0.4 is a good start
 *   DEADZONE_DEG            – ignore corrections smaller than this (prevents jitter)
 *   POSITION_WRITE_THRESHOLD– skip USB write if change is smaller than this
 *   ANGLE_TO_POS_LUT        – calibrate by recording servo positions at known angles
 */
public class Turret {

    private Servo turretServo1, turretServo2, turretServo3;

    // ── Inputs (set by callers each loop) ──────────────────────────────────
    private double targetAngle;   // signed degrees from Position.getTargetAngle()
    private double heading;       // robot heading in degrees
    private double offsetAngle;   // motion-compensation offset in degrees
    private double difPos;        // kept for API compatibility

    // ── Smoothing ──────────────────────────────────────────────────────────
    /**
     * Exponential smoothing factor for servo position.
     * 0 = never moves, 1 = instant jump.
     * 0.4 gives fast, jitter-free tracking; raise toward 1.0 for snappier response.
     */
    private static final double SMOOTHING_ALPHA = 0.4;

    // ── Limits ─────────────────────────────────────────────────────────────
    /** Ignore turret corrections smaller than this (degrees). */
    private static final double DEADZONE_DEG = 0.8;
    /** Clamp combined turret angle to this range (degrees). */
    private static final double MAX_TURRET_ANGLE = 140.0;
    /** Only push a new position to the servos when the delta exceeds this. */
    private static final double POSITION_WRITE_THRESHOLD = 0.003;

    // ── Auto-position offset (for autonomous pre-aim) ──────────────────────
    private static final double AUTO_OFFSET = 0.11765;

    // ── LUT: angle (degrees) → servo position ─────────────────────────────
    // Centre is 0.5 = 0°. Calibrate by pointing turret at known angles and
    // recording the servo position that gets it there.
    // The table MUST be sorted by ascending angle.
    private static final double[][] ANGLE_TO_POS_LUT = {
        // { angleDeg, servoPosition }
        { -160,  0.915 },
        {  -90,  0.735 },
        {  -45,  0.617 },
        {    0,  0.500 },
        {   45,  0.383 },
        {   90,  0.265 },
        {  160,  0.085 },
    };

    // ── Internal state ─────────────────────────────────────────────────────
    private double currentPosition = 0.5;   // smoothed servo position
    private double cachedPosition  = 0.5;   // last value written to hardware

    // ════════════════════════════════════════════════════════════════════════
    //  CONSTRUCTOR
    // ════════════════════════════════════════════════════════════════════════

    public Turret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        turretServo3 = hardwareMap.get(Servo.class, "turretServo3");

        applyPosition(0.5);
        currentPosition = 0.5;
        cachedPosition  = 0.5;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  SETTERS (called by Testing / TeleOp each loop before update())
    // ════════════════════════════════════════════════════════════════════════

    public void setHeading(double heading) {
        this.heading = heading;
    }

    /**
     * Accepts a signed turret angle (degrees, [-180, 180]) from
     * {@code Position.getTargetAngle()}.
     */
    public void setTargetAngle(double angle) {
        this.targetAngle = normalizeAngle(angle);
    }

    public void setOffsetAngle(double offsetAngle) {
        this.offsetAngle = offsetAngle;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  MAIN UPDATE — call once per loop
    // ════════════════════════════════════════════════════════════════════════

    public void update() {
        // 1. Combine target + motion offset, normalize, clamp
        double combinedAngle = normalizeAngle(targetAngle + offsetAngle);
        combinedAngle = clamp(combinedAngle, -MAX_TURRET_ANGLE, MAX_TURRET_ANGLE);

        // 2. Dead-zone: if turret is basically on target, hold position
        if (Math.abs(combinedAngle) < DEADZONE_DEG) {
            writePosition(currentPosition);
            return;
        }

        // 3. Look up the desired absolute servo position from the calibration table
        double targetPosition = lutLookup(combinedAngle);

        // 4. Exponential smoothing toward target — the servo's own controller handles
        //    the physical motion; we only need to damp high-frequency jitter here.
        //    Raise SMOOTHING_ALPHA for snappier tracking, lower it to reduce jitter.
        currentPosition = SMOOTHING_ALPHA * targetPosition
                        + (1.0 - SMOOTHING_ALPHA) * currentPosition;

        // 5. Write (skipped if change is below threshold — saves USB bandwidth)
        writePosition(currentPosition);
    }

    // ════════════════════════════════════════════════════════════════════════
    //  PRESET POSITIONS
    // ════════════════════════════════════════════════════════════════════════

    public void goNeutral() {
        currentPosition = 0.5;
        cachedPosition  = -1;   // force write
        applyPosition(0.5);
    }

    public void setAuto() {
        currentPosition = 0.5 + AUTO_OFFSET;
        cachedPosition  = -1;
        applyPosition(currentPosition);
    }

    public void setAutoBlue() {
        currentPosition = 0.5 - AUTO_OFFSET;
        cachedPosition  = -1;
        applyPosition(currentPosition);
    }

    // ════════════════════════════════════════════════════════════════════════
    //  GETTERS
    // ════════════════════════════════════════════════════════════════════════

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * @deprecated The LUT-based mapping replaced the linear ratio formula.
     * This value (targetAngle / 192.857…) no longer reflects actual servo
     * behaviour and is kept only for callers that depend on it.
     */
    @Deprecated
    public double getAngleRatio() {
        return targetAngle / 192.8571428571429;
    }

    public double getPosition() {
        return turretServo1.getPosition();
    }

    // ════════════════════════════════════════════════════════════════════════
    //  DEPRECATED / COMPAT
    // ════════════════════════════════════════════════════════════════════════

    /**
     * @deprecated No longer used internally. Target angle is now the signed
     * error from {@code Position.getTargetAngle()}.
     */
    @Deprecated
    public void setAngle(double angle) {
        // no-op — kept for backward compatibility
    }

    public void setDifPos(double difPos) {
        this.difPos = difPos;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  INTERNAL HELPERS
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Linearly interpolates the LUT to map an angle (degrees) to a servo
     * position (0–1).  Extrapolates flat beyond the table boundaries.
     */
    private static double lutLookup(double angleDeg) {
        // Below first entry → clamp
        if (angleDeg <= ANGLE_TO_POS_LUT[0][0]) {
            return ANGLE_TO_POS_LUT[0][1];
        }
        // Above last entry → clamp
        int last = ANGLE_TO_POS_LUT.length - 1;
        if (angleDeg >= ANGLE_TO_POS_LUT[last][0]) {
            return ANGLE_TO_POS_LUT[last][1];
        }
        // Interpolate between two surrounding entries
        for (int i = 0; i < last; i++) {
            double aLo = ANGLE_TO_POS_LUT[i][0];
            double aHi = ANGLE_TO_POS_LUT[i + 1][0];
            if (angleDeg >= aLo && angleDeg <= aHi) {
                double t = (angleDeg - aLo) / (aHi - aLo);
                return ANGLE_TO_POS_LUT[i][1] + t * (ANGLE_TO_POS_LUT[i + 1][1] - ANGLE_TO_POS_LUT[i][1]);
            }
        }
        return 0.5; // fallback (should never reach here)
    }

    /** Normalize an angle to [-180, 180]. */
    private static double normalizeAngle(double angle) {
        while (angle >  180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /** Clamp a value to [min, max]. */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Only writes to the three servos if the position has changed by more
     * than POSITION_WRITE_THRESHOLD since the last write.
     * Reduces USB-bus traffic and prevents unnecessary micro-commands.
     */
    private void writePosition(double position) {
        if (Math.abs(position - cachedPosition) > POSITION_WRITE_THRESHOLD) {
            applyPosition(position);
            cachedPosition = position;
        }
    }

    /** Low-level: sends the same position to all three turret servos. */
    private void applyPosition(double position) {
        turretServo1.setPosition(position);
        turretServo2.setPosition(position);
        turretServo3.setPosition(position);
    }
}
