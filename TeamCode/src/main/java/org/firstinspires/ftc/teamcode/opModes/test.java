package org.firstinspires.ftc.teamcode.opModes;

//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.subs.Kicker;
import org.firstinspires.ftc.teamcode.subs.Shooter;
import org.firstinspires.ftc.teamcode.subs.Spindex;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.subs.Intaker;


@TeleOp(name = "test")
public class test extends NextFTCOpMode {

    public test() {

    }



    @Override
    public void onStartButtonPressed() {


    }
}
