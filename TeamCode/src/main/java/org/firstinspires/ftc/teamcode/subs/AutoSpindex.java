package org.firstinspires.ftc.teamcode.subs;


import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class AutoSpindex implements Subsystem {


    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0;

    public int target = 0;

    private final double tick_in_degrees = 0; //TODO: DO THIS KRISHAAN

    //private DcMotorEx spindex;

    private MotorEx spindex = new MotorEx("spindex");



    private DigitalChannel LAZR;
    private boolean prev_beam_state = true;
    private int ball_count = 0;
    private long beam_rest = 0;
    public static long turn_time = 250;



    public static final AutoSpindex INSTANCE = new AutoSpindex();
    private AutoSpindex() { }

    public double tpr = 103.8 * 285/36;

    public int toggle = 0;








    public void newTurn(){
        target += tpr * 1/3;
    }

    public void newReTurn(){
        target -= tpr * 1/3;
    }
    public void zero(){
        if(toggle == 0) {
            target = 0;
            toggle++;
        }else{
            target -= tpr * .07;
            toggle = 0;
        }
    }
    public void micro(){
        target -= tpr * .14;
    }

    public void rapid(){
        target -= tpr * 5/3;
    }
    public void resetCount(){
        ball_count = 0;
    }
    public void zeroAuto(){
        target = 0;
    }
    @Override
    public void initialize(){
        spindex.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LAZR = ActiveOpMode.hardwareMap().get(DigitalChannel.class, "LAZR");
        LAZR.setMode(DigitalChannel.Mode.INPUT);
        prev_beam_state = LAZR.getState();
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

        boolean curr_beam_state = LAZR.getState();
        if(!prev_beam_state && curr_beam_state){
            ball_count++;
            beam_rest = System.currentTimeMillis();
        }
        if(beam_rest != 0 && System.currentTimeMillis() - beam_rest >= turn_time){
            newTurn();
            beam_rest = 0;
        }
        prev_beam_state = curr_beam_state;
    }

}
