package org.firstinspires.ftc.teamcode.opModes;

//import com.bylazar.configurables.annotations.Configurable;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subs.Kicker;
import org.firstinspires.ftc.teamcode.subs.Shooter;
import org.firstinspires.ftc.teamcode.subs.Spindex;
import org.opencv.dnn.Layer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
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
import static dev.nextftc.bindings.Bindings.*;

@Configurable
@TeleOp(name = "Drive")
public class FirstSteps extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(FirstSteps.class);

    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private NormalizedColorSensor colorSensor;
    public static final float gain = 20;
    private View relativeLayout;
    public int inIndex = 0;
    public int count = 0;
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

//    Button colorSens = button(() -> ( ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 2));
    //private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed()



    @Override
    public void onInit() {
        BindingManager.setLayer("short");
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

//        colorSensor.setGain(gain);



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

//        colorSens.whenBecomesTrue(()->{
//            if(count<3) {
//                Spindex.INSTANCE.newTurn();
//                count++;
//            }
//        });





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

        if(gamepad1.bWasPressed()){
            Intaker.INSTANCE.toggleIntake();
        }
//
//        if(gamepad1.dpadLeftWasPressed()){
//            count = 0;
//        }

//        if( ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < 2){
//            Intaker.INSTANCE.stopIntake();
//            Spindex.INSTANCE.newTurn();
//
//
//        }









//        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
//        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
//        // can give very low values (depending on the lighting conditions), which only use a small part
//        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
//        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
//        // colors will report at or near 1, and you won't be able to determine what color you are
//        // actually looking at. For this reason, it's better to err on the side of a lower gain
//        // (but always greater than  or equal to 1).
//
//
//        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
//        // hue, the second element (1) will contain the saturation, and the third element (2) will
//        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
//        // for an explanation of HSV color.
//        final float[] hsvValues = new float[3];
//
//        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
//
//
//        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
//        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
//        // the values you get from ColorSensor are dependent on the specific sensor you're using.
//
//        // If possible, turn the light on in the beginning (it might already be on anyway,
//        // we just make sure it is if we can).
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }
//
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
//         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
//         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
//         * for an explanation of HSV color. */
//
//        // Update the hsvValues array by passing it to Color.colorToHSV()
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        telemetry.addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
//        telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0])
//                .addData("Saturation", "%.3f", hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
//
//
//        if (colorSensor instanceof DistanceSensor) {
//            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
//        }
//        // Change the Robot Controller's background color to match the color detected by the color sensor.
//



        telemetry.update();






    }


}





