package org.firstinspires.ftc.teamcode.subs;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intaker implements Subsystem {
    public static final Intaker INSTANCE = new Intaker();
    private Intaker() { }
    private MotorEx intake = new MotorEx("intake").brakeMode();

    public final Command run = new InstantCommand(() -> intake.setPower(1.0)).requires(this);
    public final Command stop = new InstantCommand(() -> intake.setPower(0.0)).requires(this);
    public final Command toggle = new InstantCommand(() -> {
        if(intake.getPower() != 0){
            intake.setPower(0.0);
        }else{
            intake.setPower(1.0);
        }
    }).requires(this);



    @Override
    public void periodic(){


    }
}
