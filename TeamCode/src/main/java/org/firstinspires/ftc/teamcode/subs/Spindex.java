package org.firstinspires.ftc.teamcode.subs;


import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class Spindex implements Subsystem {

    public static PIDCoefficients coefficients = new PIDCoefficients(0.01,0.0,0.0);

    public static double targetPos = 0;
    public static double pos = 0;

    public static double offset = 0;

    public static final Spindex INSTANCE = new Spindex();
    private Spindex() { }

    private MotorEx spindex = new MotorEx("spindex").zeroed();

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(coefficients)
            .build();

    public Command turnTo(double position) {
        targetPos = position;
        return new RunToPosition(controlSystem, position + offset, new KineticState(10.0));


    }

    public final Command reset = new InstantCommand(() ->{
        spindex.zero();
    }).requires(this);





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
}
