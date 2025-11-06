package org.firstinspires.ftc.teamcode.opModes;

//import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.subs.Kicker;
import org.firstinspires.ftc.teamcode.subs.Shooter;
import org.firstinspires.ftc.teamcode.subs.Spindex;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.subs.Intaker;

@Configurable
@TeleOp(name = "Baby's First Teleop")
public class FirstSteps extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(FirstSteps.class);
    //private static final Logger log = LoggerFactory.getLogger(FirstSteps.class);

    public static double posit1 = 0;
    public static double posit2 = 1425.1 * 16/24 * 2/6;
    public static double posit3 = 1425.1 * 16/24 * 4/6;

    public static double posit4 = 1425.1 * 16/24 * 3/6;
    public static double posit5 = 1425.1 * 16/24 * 5/6;
    public static double posit6 = 1425.1 * 16/24 * 1/6;

    public static double position = 0;



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

    }
    @Override
    public void onWaitForStart() {
        telemetry.addData("pos", Spindex.pos);
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

        Gamepads.gamepad1().rightTrigger()
                .greaterThan(.0167)
                .whenBecomesTrue(Shooter.INSTANCE.run)
                .whenBecomesFalse(Shooter.INSTANCE.stop);


        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.167)
                .whenBecomesTrue(Kicker.INSTANCE.toShooter)
                .whenBecomesFalse(Kicker.INSTANCE.toSpindex);


        Gamepads.gamepad2().circle()
                .whenBecomesTrue(Spindex.INSTANCE.turnIntake(posit1));

        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(Spindex.INSTANCE.turnIntake(posit4));

        Gamepads.gamepad2().triangle()
                .whenBecomesTrue(Spindex.INSTANCE.turnIntake(posit2));

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(Spindex.INSTANCE.turnIntake(posit5));

        Gamepads.gamepad2().square()
                .whenBecomesTrue(Spindex.INSTANCE.turnIntake(posit3));

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(Spindex.INSTANCE.turnIntake(posit6));








    }

    @Override
    public void onUpdate(){

       telemetry.addData("pos", -Shooter.INSTANCE.velocity);
       telemetry.addData("targetpos", 2000);
       telemetry.addData("maybe", inIndex);
       telemetry.update();


    }
}
