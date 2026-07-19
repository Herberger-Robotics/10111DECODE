package org.firstinspires.ftc.teamcode.subs;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Save;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;


import java.util.List;

@Configurable
public class TurretSubsystem implements Subsystem {

    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    private TurretSubsystem() {}

    public static double INCREMENT_AMOUNT = 0.05;
    public static double MIN_POSITION     = 0.0;
    public static double MAX_POSITION     = 1.0;

    private final ServoEx hood1  = new ServoEx("hood1");
    private final ServoEx hood2 = new ServoEx("hood2");

    private double currentPosition = 0.0;

    // ── Servo geometry — tune these to match your Axon Mini setup ─────────
    // GEAR_RATIO: ratio between servo output and turret output shaft
    private static final double GEAR_RATIO  = 0.9923076923;
    // SERVO_RANGE: the usable degree range of the Axon Mini (check your programmer)
    private static final double SERVO_RANGE = 326.0;
    // linearFactor: corrects for slight nonlinearity across the servo range
    private static final double LINEAR_FACTOR = 1.0 + (2.0 / 90.0);
    // ─────────────────────────────────────────────────────────────────────

    // ── Turret limits ─────────────────────────────────────────────────────
    private static final double TURRET_MIN_DEG = -85;
    private static final double TURRET_MAX_DEG =  85;
    // ─────────────────────────────────────────────────────────────────────

    // ── Field targeting ───────────────────────────────────────────────────
    public static double  distance         = 0.0;
    public static boolean AUTO_AIM         = true;
    public static boolean PINPOINT_LOST    = false;

    public static boolean ll    = false;

    public static double  targetTurretAng  = 0.0;


    public static double  LL_CORRECTION_GAIN      = 0.5;
    public static double  LL_CORRECTION_THRESHOLD = 5; // degrees — how close before LL kicks in
    public static int     LL_TARGET_TAG_ID        = 24;   // -1 = any tag, or set specific ID

    private Limelight3A limelight;
    private boolean     limelightTagVisible = false;
    private double      limelightTx         = 0.0;
    // ─────────────────────────────────────────────────────────────────────
    private double        slewedTurretAng  = 0.0;  // actual angle sent to servo, slew-limited

    // Max degrees the servo target can change per periodic() call.
    // Lower = smoother/slower turret, less gyroscopic stress on servos.
    // At ~50 loops/sec, 3.0°/loop = ~150°/sec max slew rate.
    public static double MAX_SLEW_DEG_PER_LOOP = 3.0;

    public Pose goal      = new Pose(144, 144);
    public Pose exactGoal = new Pose(144, 144);
    public static Pose resetPose = new Pose(8.3, 7.8, Math.toRadians(180));

    private Pose    lastKnownPose       = new Pose(0, 0, 0);
    private boolean pinpointWarningSent = false;
    // ─────────────────────────────────────────────────────────────────────

    // ── Hardware ──────────────────────────────────────────────────────────
    // ServoGroup drives both servos to the same position in one call.
    // If one servo is mounted mirrored, call setReversed(true) on it below.
    private final ServoEx    servoLeft   = new ServoEx("servoLeft");
    private final ServoEx    servoRight  = new ServoEx("servoRight");
    private final ServoGroup turretServos = new ServoGroup(servoLeft, servoRight);
    // ─────────────────────────────────────────────────────────────────────

    // ── NextFTC lifecycle ─────────────────────────────────────────────────

    @Override
    public void initialize() {
        // If your servos are mounted mirrored (one faces the other),
        // uncomment this so ServoGroup automatically flips one:
        // servoRight.setReversed(true);
        setPose(Save.side);
        targetTurretAng = 0.0;
        slewedTurretAng = 0.0;

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
        setPosition(0.0);

        update();
    }

    @Override
    public void periodic() {
        // 1. Field targeting
        ActiveOpMode.telemetry().addData("Hood left  pos", String.format("%.3f", hood1.getPosition()));
        ActiveOpMode.telemetry().addData("Hood right pos", String.format("%.3f", hood2.getPosition()));
        ActiveOpMode.telemetry().addData("Hood target pos", String.format("%.3f", currentPosition));

        if (AUTO_AIM) {
            turretLoop();
            /*if(ll) {
                readLimelight();
                if (limelightTagVisible) {
                    double error = Math.abs(targetTurretAng - slewedTurretAng);
                    if (error < LL_CORRECTION_THRESHOLD) {
                        // tx > 0 means tag is to the RIGHT, turret needs to go right
                        targetTurretAng = clamp(
                                targetTurretAng + limelightTx * LL_CORRECTION_GAIN,
                                TURRET_MIN_DEG,
                                TURRET_MAX_DEG
                        );
                    }
                }
            }*/
        }

        // 2. Limelight correction — trims residual error once turret is near target


        // 3. Distance to goal
        distanceCalc();

        // 3. Pinpoint lost warning
        if (PINPOINT_LOST && !pinpointWarningSent) {
            // RGBLight.INSTANCE.pinpointWarning().schedule();
            pinpointWarningSent = true;
        } else if (!PINPOINT_LOST) {
            pinpointWarningSent = false;
        }

        // 4. Write angle to servos
        update();

        if(distance <= 3){
            setPosition(0.45);
        }
        else if(3.75 < distance && distance <=4.5 ) {
            setPosition(.4);
        }
        else{
            setPosition(0.45);
        }


        // 5. Telemetry
        ActiveOpMode.telemetry().addData("Turret target (°)",    String.format("%.1f", targetTurretAng));
        ActiveOpMode.telemetry().addData("Turret slewed (°)",    String.format("%.1f", slewedTurretAng));
        ActiveOpMode.telemetry().addData("Turret servo pos",     String.format("%.4f", turretAngleToServo(slewedTurretAng)));
        ActiveOpMode.telemetry().addData("Turret auto aim",      AUTO_AIM);
        ActiveOpMode.telemetry().addData("LL tag visible",       limelightTagVisible);
        ActiveOpMode.telemetry().addData("LL tx (°)",            String.format("%.2f", limelightTx));
        ActiveOpMode.telemetry().addData("Turret distance (ft)", String.format("%.2f", distance));
        ActiveOpMode.telemetry().addData("Pinpoint lost",        PINPOINT_LOST);
    }

    // ── Servo math (from sample code) ────────────────────────────────────

    /**
     * Converts a turret angle in degrees to a servo position (0.0 – 1.0).
     * 0.5 = center, matches servo center position.
     */
    private double turretAngleToServo(double angleDeg) {
        return (-angleDeg * LINEAR_FACTOR) * (GEAR_RATIO / SERVO_RANGE) + 0.5;
    }

    /** Clamp and write current targetTurretAng to both servos */
    private void update() {
        double clamped = clamp(targetTurretAng, TURRET_MIN_DEG, TURRET_MAX_DEG);

        // Slew rate limiter — slewedTurretAng tracks toward clamped target
        // but can only move MAX_SLEW_DEG_PER_LOOP degrees per call
        double delta = clamped - slewedTurretAng;
        delta = clamp(delta, -MAX_SLEW_DEG_PER_LOOP, MAX_SLEW_DEG_PER_LOOP);
        slewedTurretAng += delta;
        double servoPos = turretAngleToServo(slewedTurretAng);
        // Clamp servo position to the servo's own min/max output
        servoPos = clamp(servoPos,
                turretAngleToServo(TURRET_MAX_DEG),
                turretAngleToServo(TURRET_MIN_DEG));
        turretServos.setPosition(servoPos);
    }

    // ── Limelight ─────────────────────────────────────────────────────────

    private void readLimelight() {
        limelightTagVisible = false;
        limelightTx         = 0.0;

        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null) return;   // ← only null check, drop isValid()

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return;

        for (LLResultTypes.FiducialResult tag : tags) {
            boolean idMatch = (LL_TARGET_TAG_ID == -1) ||
                    (tag.getFiducialId() == LL_TARGET_TAG_ID);
            if (idMatch) {
                limelightTx         = tag.getTargetXDegrees();
                limelightTagVisible = true;
                break;
            }
        }
    }

    // ── Field targeting ───────────────────────────────────────────────────

    public void turretLoop() {
        if (PedroComponent.follower() == null) return;
        Pose currentPose = PedroComponent.follower().getPose();

        if (isPoseValid(currentPose)) {
            lastKnownPose = currentPose;
            PINPOINT_LOST = false;
        } else {
            PINPOINT_LOST = true;
            currentPose = lastKnownPose;
        }

        double dx = goal.getX() - currentPose.getX();
        double dy = goal.getY() - currentPose.getY();

        double goalFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg   = Math.toDegrees(currentPose.getHeading());
        double desired      = normalizeAngle(goalFieldDeg - headingDeg);

        targetTurretAng = clamp(desired, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    public void turretAuto(Pose p) {
        double dx = goal.getX() - p.getX();
        double dy = goal.getY() - p.getY();

        double goalFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg   = Math.toDegrees(p.getHeading());
        double desired      = normalizeAngle(goalFieldDeg - headingDeg);

        targetTurretAng = clamp(desired, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    public void setPose(int alliance) {
        if (alliance == 0) {
            goal      = new Pose(139, 129);
            exactGoal = new Pose(144, 144);
            resetPose = new Pose(8.3, 7.8, Math.toRadians(180));
        } else {
            goal      = new Pose(9.2, 141.88);
            exactGoal = new Pose(144, 144).mirror();
            resetPose = new Pose(8.3, 7.8, Math.toRadians(180)).mirror();
        }
    }

    private void distanceCalc() {
        if (PedroComponent.follower() == null) return;
        Pose pose = PINPOINT_LOST ? lastKnownPose : PedroComponent.follower().getPose();
        distance = Math.hypot(
                exactGoal.getX() - pose.getX(),
                exactGoal.getY() - pose.getY()
        ) / 12.0;
    }

    private boolean isPoseValid(Pose pose) {
        return !(pose.getX() == 0 && pose.getY() == 0 && pose.getHeading() == 0);
    }

    // ── Auto aim toggles ──────────────────────────────────────────────────

    public void enableAutoAim()  { AUTO_AIM = true;  }
    public void disableAutoAim() { AUTO_AIM = false;
        targetTurretAng = 0.0;
    }

    public void overrideTeleOp() {
        disableAutoAim();
        targetTurretAng = 0.0;
    }

    // ── Public helpers ────────────────────────────────────────────────────

    public void setTargetDegrees(double angleDeg) {
        AUTO_AIM = false;
        targetTurretAng = clamp(angleDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    public void shiftTargetDegrees(double deltaDeg) {
        targetTurretAng = clamp(targetTurretAng + deltaDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    // ── Commands ──────────────────────────────────────────────────────────

    public Command goToAngle(double angleDeg) {
        return new InstantCommand(() -> setTargetDegrees(angleDeg))
                .requires(this)
                .named("Turret goTo " + angleDeg + "°");
    }

    public Command center() {
        return goToAngle(0.0);
    }

    public Command autoAimOn() {
        return new InstantCommand(this::enableAutoAim)
                .requires(this)
                .named("Turret AutoAim ON");
    }

    public Command autoAimOff() {
        return new InstantCommand(this::disableAutoAim)
                .requires(this)
                .named("Turret AutoAim OFF");
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    public double normalizeAngle(double angDeg) {
        double ang = angDeg % 360.0;
        if (ang >  180.0) ang -= 360.0;
        if (ang < -180.0) ang += 360.0;
        return ang;
    }



    //HOOOOOOOOOOOOOOOOOOOOOOOOOOD

    public Command goTo(double position) {
        return new InstantCommand(() -> setPosition(position))
                .requires(this)
                .named("Hood goTo " + position);
    }


    public Command increment() {
        return new InstantCommand(() -> setPosition(currentPosition + INCREMENT_AMOUNT))
                .requires(this)
                .named("Hood increment");
    }


    public Command decrement() {
        return new InstantCommand(() -> setPosition(currentPosition - INCREMENT_AMOUNT))
                .requires(this)
                .named("Hood decrement");
    }


    public double getPosition() {
        return currentPosition;
    }





    private void setPosition(double position) {
        currentPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));

        hood1.setPosition(currentPosition);
        hood2.setPosition(currentPosition);

    }
}