package org.firstinspires.ftc.teamcode.subs;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }
    private MotorEx shooter = new MotorEx("shooter").brakeMode();

    public final Command run = new InstantCommand(() -> shooter.setPower(1.0)).requires(this);
    public final Command stop = new InstantCommand(() -> shooter.setPower(0.0)).requires(this);
    public final Command toggle = new InstantCommand(() -> {
        if(shooter.getPower() != 0){
            shooter.setPower(0.0);
        }else{
            shooter.setPower(1.0);
        }
    }).requires(this);



    @Override
    public void periodic(){


    }
}
