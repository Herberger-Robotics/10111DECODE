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
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPositions;

@Configurable
public class Hood implements Subsystem {


    public static final Hood INSTANCE = new Hood();
    private Hood() { }

    public double velocity = 0;


    private ServoEx hood1 = new ServoEx("hood1");
    private ServoEx hood2 = new ServoEx("hood2");






    public Command base = new SetPositions(
            hood1.to(0.02),
            hood2.to(.98))
            .requires(this);

    public Command turn = new SetPositions(
            hood1.to(0.15),
            hood2.to(.85))
            .requires(this);


    @Override
    public void periodic(){

    }
}

