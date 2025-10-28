package org.firstinspires.ftc.teamcode.subs;


import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class Spindex implements Subsystem {

    public static PIDCoefficients coefficients = new PIDCoefficients(0.01,0.0,0.0);

    public static int posit1 = 0;
    public static int posit2 = 300;
    public static int posit3 = 600;

    public static final Spindex INSTANCE = new Spindex();
    private Spindex() { }

    private MotorEx spindex = new MotorEx("spindex");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(coefficients)
            .build();

    public Command pos1 = new RunToPosition(controlSystem, posit1).requires(this);
    public Command pos2 = new RunToPosition(controlSystem, posit2).requires(this);
    public Command pos3 = new RunToPosition(controlSystem, posit3).requires(this);

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

        spindex.setPower(controlSystem.calculate(spindex.getState()));



    }
}
