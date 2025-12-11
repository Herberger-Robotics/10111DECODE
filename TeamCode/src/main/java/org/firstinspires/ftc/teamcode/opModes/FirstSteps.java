package org.firstinspires.ftc.teamcode.opModes;

//import com.bylazar.configurables.annotations.Configurable;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.subs.Kicker;
import org.firstinspires.ftc.teamcode.subs.Shooter;
import org.firstinspires.ftc.teamcode.subs.Spindex;
import org.opencv.dnn.Layer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import kotlin.jvm.functions.Function0;

import org.firstinspires.ftc.teamcode.subs.Intaker;

@Configurable
@TeleOp(name = "Drive")
public class FirstSteps extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(FirstSteps.class);

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    public int inIndex = 0;
    public FirstSteps() {
        addComponents(
                new SubsystemComponent(
                        Shooter.INSTANCE,
                        Intaker.INSTANCE,
                        Kicker.INSTANCE,
                        Spindex.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left").brakeMode().reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right").brakeMode();
    //for my eventual evil field centric
    //private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed()


    @Override
    public void onInit() {
        BindingManager.setLayer("short");



    }
    @Override
    public void onWaitForStart() {
      //  telemetry.addData("pos", Spindex.pos);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {


        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();

        Gamepads.gamepad1().rightBumper()
                        .whenBecomesTrue(()-> driverControlled.setScalar(0.3))
                        .whenBecomesFalse(() -> driverControlled.setScalar(1));

        Gamepads.gamepad1().a()
                .whenBecomesTrue(Intaker.INSTANCE.reverse)
                .whenBecomesFalse(Intaker.INSTANCE.stop);

        Gamepads.gamepad1().b()
                .whenBecomesTrue(Intaker.INSTANCE.toggle);

        Gamepads.gamepad2().square()
                .inLayer("far")
                .whenBecomesTrue(Shooter.INSTANCE.start)
                .inLayer("short")
                .whenBecomesTrue(Shooter.INSTANCE.startclose);


        Gamepads.gamepad2().triangle()
                        .whenBecomesTrue(Shooter.INSTANCE.stop);





        Gamepads.gamepad1().dpadUp()
                        .whenBecomesTrue(() -> BindingManager.setLayer("far"));

        Gamepads.gamepad1().dpadDown()
                        .whenBecomesTrue(() -> BindingManager.setLayer("short"));





    }

    @Override
    public void onUpdate(){

        if(gamepad2.circleWasPressed()){
            Spindex.INSTANCE.newTurn();
        }

        if(gamepad1.squareWasPressed()){
            Spindex.INSTANCE.newReTurn();
        }

        if(gamepad1.triangleWasPressed()){
            Spindex.INSTANCE.zeroAuto();
        }

        if(gamepad1.leftBumperWasPressed()){
            Spindex.INSTANCE.micro();
        }



       panelsTelemetry.addData("shooterpos", -Shooter.INSTANCE.velocity);
       panelsTelemetry.addData("targetshooterpos", 1820);
       panelsTelemetry.update();

       telemetry.addData("shooterpos", -Shooter.INSTANCE.velocity);
       telemetry.addData("targetshooterpos", 1820);
       telemetry.update();




    }

}
