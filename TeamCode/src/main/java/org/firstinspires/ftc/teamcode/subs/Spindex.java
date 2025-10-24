package org.firstinspires.ftc.teamcode.subs;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Spindex implements Subsystem {
    public static final Spindex INSTANCE = new Spindex();
    private Spindex() { }

    private MotorEx spindex = new MotorEx("spindex");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)

            .build();

    public Command pos1 = new RunToPosition(controlSystem, 0).requires(this);
    public Command pos2 = new RunToPosition(controlSystem, 500).requires(this);
    public Command pos3 = new RunToPosition(controlSystem, 1200).requires(this);

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
