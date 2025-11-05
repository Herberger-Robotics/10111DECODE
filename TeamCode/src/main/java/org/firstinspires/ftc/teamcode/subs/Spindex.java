package org.firstinspires.ftc.teamcode.subs;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class Spindex implements Subsystem {
    public static PIDCoefficients coefficients = new PIDCoefficients(0.01,0.0,0.0);

    public static double targetPos = 0;
    public static double pos = 0;

    public static double offset = 0;

    public int intakeIndex = 0;
    public int shooterIndex = 0;

    private final double[] intakePositions = {
            160, 480, 800  // Intake 1, 2, 3
    };

    private final double[] shooterPositions = {
            0, 320, 640  // Shooter 1, 2, 3
    };

    public static final Spindex INSTANCE = new Spindex();
    private Spindex() { }

    private MotorEx spindex = new MotorEx("spindex");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(coefficients)
            .build();
    public Command turnTo(double position) {
        targetPos = position;
        return new RunToPosition(controlSystem, position + offset, new KineticState(10.0));
    }

    public Command rightIntake(){
        intakeIndex++;
        double target = intakePositions[Math.abs((intakeIndex) % intakePositions.length)];
        return new RunToPosition(controlSystem, target, new KineticState(10));
    }

    public Command rightShooter(){
        shooterIndex++;
        double target = shooterPositions[Math.abs((shooterIndex) % shooterPositions.length)];
        return new RunToPosition(controlSystem, target, new KineticState(10));
    }
    public Command leftIntake() {
        // step backward and wrap around correctly
        intakeIndex--;
        double target = intakePositions[Math.abs((intakeIndex) % intakePositions.length)];
        return new RunToPosition(controlSystem, target, new KineticState(10));
    }

    public Command leftShooter() {
        shooterIndex--;

        double target = shooterPositions[Math.abs((shooterIndex) % shooterPositions.length)];
        return new RunToPosition(controlSystem, target, new KineticState(10));
    }

    /*public final Command rotate = new InstantCommand(() -> {
        if(shooter.getPower() != 0){
            shooter.setPower(0.0);
        }else{
            shooter.setPower(1.0);
        }
    }).requires(this);
    WIP
    */

    @Override
    public void periodic() {
        pos = spindex.getCurrentPosition();
        spindex.setPower(controlSystem.calculate(spindex.getState()));
    }
    @Override
    public void initialize(){
        spindex.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Spindex.INSTANCE.turnTo(0);

    }
}
