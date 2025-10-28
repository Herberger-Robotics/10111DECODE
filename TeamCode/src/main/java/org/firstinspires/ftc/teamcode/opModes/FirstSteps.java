package org.firstinspires.ftc.teamcode.opModes;

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

@TeleOp(name = "Baby's First Teleop")
public class FirstSteps extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(FirstSteps.class);

    public FirstSteps() {
        addComponents(
                new SubsystemComponent
                        (Intaker.INSTANCE,
                        Shooter.INSTANCE,
                        Spindex.INSTANCE,
                        Kicker.INSTANCE),
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
                .whenBecomesTrue(Intaker.INSTANCE.run)
                .whenBecomesFalse(Intaker.INSTANCE.stop);

        Gamepads.gamepad1().b()
                .whenBecomesTrue(Intaker.INSTANCE.toggle);

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Shooter.INSTANCE.toggle);

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Spindex.INSTANCE.pos1);

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(Spindex.INSTANCE.pos2);

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(Spindex.INSTANCE.pos3);



    }
}
