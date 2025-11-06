package org.firstinspires.ftc.teamcode.subs;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class Shooter implements Subsystem {
    public static PIDCoefficients coefficients = new PIDCoefficients(0.011,0.0,0.0);
    public static BasicFeedforwardParameters ffcoefficients = new BasicFeedforwardParameters(.01,0.02,0);
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    public double velocity = 0;
    private MotorEx shooter = new MotorEx("shooter").brakeMode();

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(ffcoefficients)
            .build();


    //public final Command run = new RunToVelocity(controlSystem,2000).requires(this).named("shooter on");
    //public final Command stop = new RunToVelocity(controlSystem, 0).requires(this).named("shooter off");



    public final InstantCommand start = new InstantCommand(() ->
            controlSystem.setGoal(new KineticState(0.0, (2000)))
    );

    public final InstantCommand stop = new InstantCommand(() ->
            controlSystem.setGoal(new KineticState())
    );

    @Override
    public void periodic(){
        velocity = shooter.getVelocity();
        shooter.setPower(controlSystem.calculate(shooter.getState()));
    }
}
