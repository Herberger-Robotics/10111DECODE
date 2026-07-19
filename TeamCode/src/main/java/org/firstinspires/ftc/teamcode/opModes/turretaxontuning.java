package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;

/**
 * TurretAxonTuning
 *
 * ══════════════════════════════════════════════════════
 * WHAT TO DO, IN ORDER:
 * ══════════════════════════════════════════════════════
 *
 * 1. SERVO DIRECTION
 *    Press A (center). The turret should sit at its middle position.
 *    Press DPAD RIGHT (+45°). Turret should rotate RIGHT.
 *    If it goes left, uncomment setReversed(true) on servoRight below.
 *
 * 2. ENDPOINT CHECK
 *    Press LEFT BUMPER → both servos go to 0.0 (one physical extreme)
 *    Press RIGHT BUMPER → both servos go to 1.0 (other physical extreme)
 *    Confirm the turret reaches both hard stops without grinding.
 *    If it grinds before reaching the stop, your SERVO_RANGE is too wide —
 *    lower it in the Axon programmer app and update SERVO_RANGE here.
 *
 * 3. GEAR RATIO
 *    Press A to center. Mark the turret position with tape.
 *    Press DPAD UP to command +90°.
 *    Physically measure the actual rotation (protractor or phone level).
 *    Enter the measured degrees into MEASURED_DEG below.
 *    The "Computed GEAR_RATIO" on screen is the value to copy into TurretSubsystem.
 *    Update GEAR_RATIO here, re-run, press DPAD UP again, confirm it's now 90°.
 *
 * ══════════════════════════════════════════════════════
 * CONTROLS
 * ══════════════════════════════════════════════════════
 *   A           → center (0°, servo pos 0.5)
 *   DPAD RIGHT  → +45°
 *   DPAD LEFT   → -45°
 *   DPAD UP     → +90°   ← use this for gear ratio measurement
 *   DPAD DOWN   → -90°
 *   LEFT BUMPER → both servos to 0.0 (endpoint check)
 *   RIGHT BUMPER→ both servos to 1.0 (endpoint check)
 */
@Configurable
@TeleOp(name = "Turret Axon Tuning", group = "Tuning")
public class turretaxontuning extends NextFTCOpMode {

    // ── SET THESE ─────────────────────────────────────────────────────────
    public static double GEAR_RATIO    = 0.9923076923; // copy computed value here after step 3
    public static double SERVO_RANGE   = 326.0;        // from Axon programmer app
    public static double LINEAR_FACTOR = 1.0 + (2.0 / 90.0);

    // Step 3: after commanding +90° and physically measuring, enter actual degrees here
    public static double MEASURED_DEG  = 90.0;
    // ─────────────────────────────────────────────────────────────────────

    private final ServoEx    servoLeft    = new ServoEx("servoLeft");
    private final ServoEx    servoRight   = new ServoEx("servoRight");
    private final ServoGroup turretServos = new ServoGroup(servoLeft, servoRight);

    private double targetAngleDeg = 0.0;
    private boolean endpointMode  = false;
    private double  endpointPos   = 0.5;

    @Override
    public void onInit() {
        // If one servo is mounted mirrored, uncomment:
        // servoRight.setReversed(true);

        turretServos.setPosition(0.5);
        telemetry.addLine("Turret Axon Tuning — press Start.");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        // ── Endpoint mode — bypasses angle math ───────────────────────────
        if (lb) {
            endpointMode = true;
            endpointPos  = 0.0;
        } else if (rb) {
            endpointMode = true;
            endpointPos  = 1.0;
        } else {
            endpointMode = false;
        }

        // ── Angle commands ────────────────────────────────────────────────
        if (gamepad1.a)          targetAngleDeg =   0.0;
        if (gamepad1.dpad_right) targetAngleDeg =  45.0;
        if (gamepad1.dpad_left)  targetAngleDeg = -45.0;
        if (gamepad1.dpad_up)    targetAngleDeg =  90.0;
        if (gamepad1.dpad_down)  targetAngleDeg = -90.0;

        targetAngleDeg = Math.max(-170.0, Math.min(170.0, targetAngleDeg));

        // ── Write to servos ───────────────────────────────────────────────
        double servoPos;
        if (endpointMode) {
            servoPos = endpointPos;
            turretServos.setPosition(servoPos);
        } else {
            servoPos = turretAngleToServo(targetAngleDeg);
            servoPos = Math.max(0.0, Math.min(1.0, servoPos));
            turretServos.setPosition(servoPos);
        }

        // ── Gear ratio calculator ─────────────────────────────────────────
        // Commanding +90°. Enter MEASURED_DEG = actual physical degrees turned.
        // Computed GEAR_RATIO = what you copy into TurretSubsystem.
        double computedGearRatio = MEASURED_DEG / 90.0;

        // ── Telemetry ─────────────────────────────────────────────────────
        telemetry.addLine("=== TURRET AXON TUNING ===");
        telemetry.addData("Mode", endpointMode
                ? (endpointPos == 0.0 ? "ENDPOINT → 0.0 (LB)" : "ENDPOINT → 1.0 (RB)")
                : "ANGLE");
        telemetry.addLine("──────────────────────────");
        telemetry.addData("Target angle (°)", String.format("%.1f", targetAngleDeg));
        telemetry.addData("Servo position",   String.format("%.4f", servoPos));
        telemetry.addLine("──────────────────────────");
        telemetry.addLine("GEAR RATIO CALCULATOR:");
        telemetry.addData("  Commanded (°)",       90.0);
        telemetry.addData("  MEASURED_DEG",        MEASURED_DEG);
        telemetry.addData("  Computed GEAR_RATIO", String.format("%.7f", computedGearRatio));
        telemetry.addLine("  ^ Copy this into TurretSubsystem + GEAR_RATIO above");
        telemetry.addLine("──────────────────────────");
        telemetry.addLine("A=0°  DPAD R=+45  L=-45  U=+90  D=-90");
        telemetry.addLine("LB = servo 0.0    RB = servo 1.0");
        telemetry.update();
    }

    private double turretAngleToServo(double angleDeg) {
        return (-angleDeg * LINEAR_FACTOR) * (GEAR_RATIO / SERVO_RANGE) + 0.5;
    }
}