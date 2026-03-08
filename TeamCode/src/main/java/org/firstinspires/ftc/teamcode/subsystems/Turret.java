package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 3-servo turret driven by Continuous-Rotation Servos (CRServos).
 *
 * CRServos are velocity-controlled: {@code setPower(p)} where p ∈ [−1, 1]
 * sets spin speed and direction — they have NO built-in position control.
 * The correct approach is proportional control:
 *
 *     power = kP × angle_error   (clamped to [−1, 1])
 *
 * The {@code targetAngle} supplied by {@code Position.getTargetAngle()} is
 * already the signed angular error (how far off the turret is), so it feeds
 * directly into the P-controller each loop.
 *
 * Tuning guide:
 *   kP              – proportional gain; raise for faster response, lower if oscillating
 *   DEADZONE_DEG    – errors smaller than this stop the motors (prevents jitter)
 *   MAX_TURRET_ANGLE– clamp combined angle to this range (degrees)
 *   POWER_THRESHOLD – skip setPower() call if change is below this (saves USB bandwidth)
 *
 * Hardware configuration: in the Robot Controller config the three servo
 * ports must be set to "Continuous Servo" mode and named turretServo1/2/3.
 * If the turret spins the wrong way, negate kP.
 */
public class Turret {

    private CRServo turretServo1, turretServo2, turretServo3;

    // ── Inputs (set by callers each loop) ──────────────────────────────────
    private double targetAngle;   // signed degrees from Position.getTargetAngle()
    private double heading;       // robot heading in degrees (kept for API compat)
    private double offsetAngle;   // motion-compensation offset in degrees
    private double difPos;        // kept for API compatibility

    // ── Proportional gain ──────────────────────────────────────────────────
    /**
     * P-gain converting angle error (degrees) to CRServo power.
     * At kP = 0.01: a 100° error → power 1.0 (full speed).
     * If the turret rotates in the wrong direction, flip the sign.
     */
    private static final double kP = 0.01;

    // ── Limits ─────────────────────────────────────────────────────────────
    /** Errors smaller than this (degrees) stop the turret — prevents jitter. */
    private static final double DEADZONE_DEG = 1.5;
    /** Clamp combined turret angle to this range (degrees). */
    private static final double MAX_TURRET_ANGLE = 140.0;
    /** Skip setPower() call when the new power is within this of the last written value. */
    private static final double POWER_THRESHOLD = 0.01;

    // ── Auto pre-aim angles ────────────────────────────────────────────────
    /**
     * Pre-aim target angles (degrees) for autonomous red/blue modes.
     * After calling setAuto()/setAutoBlue(), drive update() in a loop
     * until the turret reaches the target.
     */
    private static final double AUTO_ANGLE_RED  =  45.0;
    private static final double AUTO_ANGLE_BLUE = -45.0;

    // ── Internal state ─────────────────────────────────────────────────────
    private double currentPower = 0.0;          // last power sent to the CRServos
    private double cachedPower  = Double.NaN;   // sentinel — forces write on first call

    // ════════════════════════════════════════════════════════════════════════
    //  CONSTRUCTOR
    // ════════════════════════════════════════════════════════════════════════

    public Turret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");
        turretServo3 = hardwareMap.get(CRServo.class, "turretServo3");

        applyPower(0.0);
    }

    // ════════════════════════════════════════════════════════════════════════
    //  SETTERS (called by Testing / TeleOp each loop before update())
    // ════════════════════════════════════════════════════════════════════════

    public void setHeading(double heading) {
        this.heading = heading;
    }

    /**
     * Accepts a signed turret angle error (degrees, [-180, 180]) from
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

        // 2. Dead-zone: close enough → stop
        if (Math.abs(combinedAngle) < DEADZONE_DEG) {
            writePower(0.0);
            return;
        }

        // 3. Proportional control: drive power proportional to angle error.
        //    CRServos are velocity-controlled so this naturally slows down
        //    as the turret approaches its target.
        double power = clamp(kP * combinedAngle, -1.0, 1.0);

        // 4. Write (skipped when change is below threshold — saves USB bandwidth)
        writePower(power);
    }

    // ════════════════════════════════════════════════════════════════════════
    //  PRESET POSITIONS
    // ════════════════════════════════════════════════════════════════════════

    /** Immediately stops the turret and resets the target angle to 0. */
    public void goNeutral() {
        targetAngle = 0;
        cachedPower = Double.NaN;   // force write
        applyPower(0.0);
    }

    /**
     * Pre-aims the turret for autonomous red alliance.
     * Must be followed by calling {@code update()} in a loop until on target.
     */
    public void setAuto() {
        targetAngle = AUTO_ANGLE_RED;
    }

    /**
     * Pre-aims the turret for autonomous blue alliance.
     * Must be followed by calling {@code update()} in a loop until on target.
     */
    public void setAutoBlue() {
        targetAngle = AUTO_ANGLE_BLUE;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  GETTERS
    // ════════════════════════════════════════════════════════════════════════

    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * @deprecated The proportional power model replaced the linear ratio formula.
     * Kept only for callers that depend on it.
     */
    @Deprecated
    public double getAngleRatio() {
        return targetAngle / 192.8571428571429;
    }

    /** Returns the last power value written to the turret CRServos. */
    public double getPosition() {
        return currentPower;
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
     * Only writes to the three CRServos if the power has changed by more
     * than POWER_THRESHOLD since the last write.
     * Reduces USB-bus traffic and prevents unnecessary repeated commands.
     */
    private void writePower(double power) {
        if (Double.isNaN(cachedPower) || Math.abs(power - cachedPower) > POWER_THRESHOLD) {
            applyPower(power);
            cachedPower = power;
        }
    }

    /** Low-level: sends the same power to all three turret CRServos. */
    private void applyPower(double power) {
        turretServo1.setPower(power);
        turretServo2.setPower(power);
        turretServo3.setPower(power);
        currentPower = power;
    }
}
