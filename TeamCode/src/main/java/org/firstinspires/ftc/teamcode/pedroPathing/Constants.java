package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//forward 7.25
//strafe 1
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-28.93)
            .lateralZeroPowerAcceleration(-66.92)
            .translationalPIDFCoefficients(new PIDFCoefficients(.1,0,0.0001,0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.02,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.013,0.0001,0.0005,0.6,0.01))
            .centripetalScaling(0.001)
            .mass(13.15);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 0.85);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)

                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.163)
            .strafePodX(0.482)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    //61.64
    //61.98
    public static MecanumConstants driveConstants = new MecanumConstants()
            //78
            .xVelocity(83.33)
            //64
            .yVelocity(64.46)
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")

            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
}
