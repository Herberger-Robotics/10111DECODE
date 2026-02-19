package org.firstinspires.ftc.teamcode.subs;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class Spindex implements Subsystem {

//    public static PIDCoefficients coefficients = new PIDCoefficients(0.015,0.0,0.0);
//
//    public static double targetPos = 0;
//    public static double pos = 0;
//
//    public static double offset = 0;
//
//    public static double move = 160;


    private PIDController controller;

    public static double p = 0.00375, i = 0, d = 0.0001;
    public static double f = 0;

    public int target = 0;

    private final double tick_in_degrees = 0; //TODO: DO THIS KRISHAAN

    //private DcMotorEx spindex;

    private MotorEx spindex = new MotorEx("spindex");


    public static final Spindex INSTANCE = new Spindex();
    private Spindex() { }

    public double tpr = 1425.1 * 16/24;

    public int toggle = 0;


//    private ControlSystem controlSystem = ControlSystem.builder()
//
//            .posPid(coefficients)
//            .build();
//    public Command turnIntake(double target) {
//        targetPos = target;
//        return new RunToPosition(controlSystem, target, new KineticState(10.0));
//    }
//
//    public Command turn() {
//        double target = spindex.getCurrentPosition() + (1425.1 * 16/24 * 2/6);
//        return new RunToPosition(controlSystem, target, new KineticState(10.0));
//    }







    /*public final Command rotate = new InstantCommand(() -> {
        if(shooter.getPower() != 0){
            shooter.setPower(0.0);
        }else{
            shooter.setPower(1.0);
        }
    }).requires(this);
    WIP
    */

    public void newTurn(){
        target -= tpr * 1/3;
    }

    public void newReTurn(){
        target += tpr * 1/3;
    }
    public void zero(){
        if(toggle == 0) {
            target = 0;
            toggle++;
        }else{
            target += tpr * .07;
            toggle = 0;
        }
    }
    public void micro(){
        target += tpr * .14;
    }

    public void rapid(){
        target += tpr * 5/3;
    }

    public void zeroAuto(){
        target = 0;
    }
    @Override
    public void initialize(){
        spindex.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        controller = new PIDController(p,i,d);

    }

    @Override
    public void periodic() {
      //  pos = spindex.getCurrentPosition();

       // spindex.setPower(controlSystem.calculate(spindex.getState()));
        controller.setPID(p,i,d);
        double spindexPos = spindex.getCurrentPosition();
        double pid = controller.calculate(spindexPos, target);
        double ff = 0;

        double power = pid + ff;

        spindex.setPower(power);
    }

}
