package org.firstinspires.ftc.teamcode.subs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class Kicker implements Subsystem {
    public static final Kicker INSTANCE = new Kicker();
    private Kicker() { }
    private ServoEx left_kicker = new ServoEx("left_kicker");
    private ServoEx right_kicker = new ServoEx("right_kicker");


    public Command toShooter = new SetPositions(
            left_kicker.to(0.34),
            right_kicker.to(0.65))
            .requires(this);

    public Command toSpindex = new SetPositions(
            left_kicker.to(0.04),
            right_kicker.to(0.94))
            .requires(this);



}
