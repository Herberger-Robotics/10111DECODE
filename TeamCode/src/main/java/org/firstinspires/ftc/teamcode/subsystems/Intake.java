package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }
    private MotorEx intake = new MotorEx("intake");

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
