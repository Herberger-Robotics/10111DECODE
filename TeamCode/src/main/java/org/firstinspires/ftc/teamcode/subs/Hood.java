package org.firstinspires.ftc.teamcode.subs;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Hood implements Subsystem {

    public static final Hood INSTANCE = new Hood();
    private Hood() {}

    public static double INCREMENT_AMOUNT = 0.05;
    public static double MIN_POSITION     = 0.0;
    public static double MAX_POSITION     = 1.0;

    private final ServoEx servoLeft  = new ServoEx("hood1");
    private final ServoEx servoRight = new ServoEx("hood2");

    private double currentPosition = 0.0;

    //.3 for 35 .4 for 40 .5 for 45

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


    @Override
    public void initialize() {
        setPosition(0.0);
    }

    @Override
    public void periodic() {
        ActiveOpMode.telemetry().addData("Hood left  pos", String.format("%.3f", servoLeft.getPosition()));
        ActiveOpMode.telemetry().addData("Hood right pos", String.format("%.3f", servoRight.getPosition()));
        ActiveOpMode.telemetry().addData("Hood target pos", String.format("%.3f", currentPosition));
    }



    private void setPosition(double position) {
        currentPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));

        servoLeft.setPosition(currentPosition);
        servoRight.setPosition(currentPosition);

    }
}