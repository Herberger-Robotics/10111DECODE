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
    public static PIDCoefficients coefficients = new PIDCoefficients(0.005,0.0,0.0001);
    public static BasicFeedforwardParameters ffcoefficients = new BasicFeedforwardParameters(0.0,0.0,0.0);

    public static double closevelo = 1650;
    public static double farvelo = 2000;
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    public double velocity = 0;
    private MotorEx shooter = new MotorEx("shooter").brakeMode();
    private MotorEx shooter2 = new MotorEx("shooter2").brakeMode().reversed();


    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(ffcoefficients)
            .build();

    public final InstantCommand start = new InstantCommand(() ->
            controlSystem.setGoal(new KineticState(0.0, (farvelo)))
    );


    public final InstantCommand startclose = new InstantCommand(() ->
            controlSystem.setGoal(new KineticState(0.0, (closevelo)))
    );

    public final InstantCommand stop = new InstantCommand(() ->
            controlSystem.setGoal(new KineticState(0.0, 0))
    );


    @Override
    public void periodic(){
        velocity = shooter.getVelocity();
        shooter.setPower(controlSystem.calculate(shooter.getState()));
        shooter2.setPower(controlSystem.calculate(shooter.getState()));
    }
}
