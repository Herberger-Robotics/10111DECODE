package org.firstinspires.ftc.teamcode.opModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subs.Intaker;
import org.firstinspires.ftc.teamcode.subs.Kicker;
import org.firstinspires.ftc.teamcode.subs.Shooter;
import org.firstinspires.ftc.teamcode.subs.Spindex;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "Blitzin' Blue", preselectTeleOp = "Drive")
public class Rapid_Blue extends NextFTCOpMode {
    public Rapid_Blue() {
        addComponents(
                new SubsystemComponent(
                        Shooter.INSTANCE,
                        Intaker.INSTANCE,
                        Kicker.INSTANCE,
                        Spindex.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private Limelight3A limelight;

    int pathType = 1;
    private final Pose startPose = new Pose(17.5,  118.16, Math.toRadians(137));

    private final Pose initialFire = new Pose( 47.78,77.1,Math.toRadians(129));



    private final Pose spikeMark = new Pose( 36.406, 63, Math.toRadians(180));

    private final Pose ballMark3 = new Pose( 1.0,63,Math.toRadians(180));

    private final Pose secondSpikeMarkPos = new Pose( 40, 85.4, Math.toRadians(180));

    private final Pose secondBallMark3 = new Pose( 10.210,85.4,Math.toRadians(180));


    private final Pose lever2 = new Pose( 7.210,78.6,Math.toRadians(180));
    private final Pose backitup2 = new Pose( 29.210,77.83,Math.toRadians(180));


    private final Pose lever = new Pose(7,69,Math.toRadians(180));
    private final Pose backitup = new Pose(29.210,69,Math.toRadians(180));


    private final Pose gateIntake = new Pose(2,57.2,Math.toRadians(148.9));
    private final Pose gateIntake2 = new Pose(4,57,Math.toRadians(180));



    private final Pose gateIntake3 = new Pose(144 + 1,88.807 - 41,Math.toRadians(60)).mirror();
    private final Pose thirdSpikeMarkPos = new Pose(144 - 38.406, 88.807 - 64, Math.toRadians(0)).mirror();

    private final Pose thirdBallMark3 = new Pose(144 - 2,88.807 - 64,Math.toRadians(0)).mirror();
    //private Follower follower;


    private PathChain testPath;
    private PathChain spikeMark1;

    private PathChain newIntake1;
    private PathChain shootMark1;

    private PathChain secondSpikeMarkPath;


    private PathChain newIntake2;
    private PathChain secondShootMark1;

    private PathChain thirdSpikeMarkPath;


    private PathChain newIntake3;
    private PathChain thirdShootMark1;

    private PathChain leverPath;
    private PathChain leverPath2;

    private PathChain leverPathIntake;


    private PathChain shootGate;
    private Command scorePreload() {
        return new SequentialGroup(

                Shooter.INSTANCE.startclose,
                new FollowPath(testPath, true),

                rapid().thenWait(.5)


        );
    };


    private Command newIntakeSecondSpikeMark(){
        return new ParallelGroup(

                new FollowPath(newIntake1, true, 0.7),
                new SequentialGroup(
                        new Delay(0.9),
                        spinSpindex(),
                        new Delay(0.5),
                        spinSpindex(),
                        new Delay(.4)
                )


        );
    };

    private Command scoreSecondSpikeMark(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose,

                new FollowPath(shootMark1,true),

                rapid().thenWait(.5),
                Intaker.INSTANCE.run


        );
    };

    private Command newIntakeFirstSpikeMark(){
        return new ParallelGroup(
                new FollowPath(newIntake2, true, 0.7),

                new SequentialGroup(
                        new Delay(0.8),
                        spinSpindex(),
                        new Delay(0.3),
                        spinSpindex(),
                        new Delay(0.3)

                )
        );
    };
    private Command scoreFirstSpikeMark(){
        return new SequentialGroup(



                Shooter.INSTANCE.startclose,

                new FollowPath(secondShootMark1,true),

                rapid().thenWait(.5),
                Intaker.INSTANCE.run


        );
    };



    private Command newIntakeThirdSpikeMark(){

        return new ParallelGroup(


                new FollowPath(newIntake3, true, 0.7),
                new SequentialGroup(
                        new Delay(.2),
                        Shooter.INSTANCE.stop
                ),
                new SequentialGroup(
                        new Delay(0.8),
                        spinSpindex(),
                        new Delay(0.4),
                        spinSpindex(),
                        new Delay(0.3)

                )

        );

    };
    private Command scoreThirdSpikeMark(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose,

                new FollowPath(thirdShootMark1,true),

                rapid().thenWait(0.7),
                Intaker.INSTANCE.stop,
                Shooter.INSTANCE.stop

        );
    };



    private Command cycle1(){
        return new SequentialGroup(
                new FollowPath(leverPathIntake,true).thenWait(1),
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(0.8),
                                spinSpindex(),
                                new Delay(0.5),

                                Shooter.INSTANCE.startclose,

                                new ParallelGroup(
                                        new FollowPath(shootGate,true),
                                        new SequentialGroup(
                                                new Delay(0.4),
                                                spinSpindex().thenWait(0.2)

                                        )
                                ),
                                rapid().thenWait(0.7),
                                Shooter.INSTANCE.stop,
                                Intaker.INSTANCE.run
                        )
                )


        );
    };

    public void buildPaths(){
        testPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, initialFire))
                .setLinearHeadingInterpolation(startPose.getHeading(), initialFire.getHeading())
                .setBrakingStart(0.5)
                .build();
        spikeMark1 = follower().pathBuilder()
                .addPath(new BezierLine(initialFire,spikeMark))
                .setLinearHeadingInterpolation(initialFire.getHeading(),spikeMark.getHeading())
                .build();




        newIntake1 = follower().pathBuilder()
                .addPath(new BezierLine(spikeMark,ballMark3))
                .setLinearHeadingInterpolation(spikeMark.getHeading(),ballMark3.getHeading())
                .build();



        shootMark1 = follower().pathBuilder()
                .addPath(new BezierLine(lever, initialFire))
                .setLinearHeadingInterpolation(lever.getHeading(), initialFire.getHeading())
                .build();

        leverPath = follower().pathBuilder()
                .addPath(new BezierCurve(ballMark3, backitup, lever))
                .setLinearHeadingInterpolation(ballMark3.getHeading(), lever.getHeading())
                .build();

        leverPath2 = follower().pathBuilder()
                .addPath(new BezierCurve(secondBallMark3, backitup2, lever2))
                .setLinearHeadingInterpolation(secondBallMark3.getHeading(), lever.getHeading())
                .build();


        secondSpikeMarkPath = follower().pathBuilder()
                .addPath(new BezierLine(initialFire,secondSpikeMarkPos))
                .setLinearHeadingInterpolation(initialFire.getHeading(), secondSpikeMarkPos.getHeading())
                .build();



        newIntake2 = follower().pathBuilder()
                .addPath(new BezierLine(secondSpikeMarkPos,secondBallMark3))
                .setLinearHeadingInterpolation(secondSpikeMarkPos.getHeading(),ballMark3.getHeading())
                .build();


        secondShootMark1 = follower().pathBuilder()
                .addPath(new BezierLine(lever,initialFire))
                .setLinearHeadingInterpolation(lever.getHeading(), initialFire.getHeading())
                .build();


//        thirdSpikeMarkPath = follower().pathBuilder()
//                .addPath(new BezierLine(initialFire,thirdSpikeMarkPos))
//                .setLinearHeadingInterpolation(initialFire.getHeading(),thirdSpikeMarkPos.getHeading())
//                .build();


//        newIntake3 = follower().pathBuilder()
//                .addPath(new BezierLine(thirdSpikeMarkPos,thirdBallMark3))
//                .setLinearHeadingInterpolation(thirdSpikeMarkPos.getHeading(),ballMark3.getHeading())
//                .build();
//        thirdShootMark1 = follower().pathBuilder()
//                .addPath(new BezierCurve(thirdBallMark3, new Pose(144 - 30.210,88.807 - 56,Math.toRadians(0)).mirror(), initialFire))
//                .setLinearHeadingInterpolation(thirdBallMark3.getHeading(), initialFire.getHeading())
//                .build();

        leverPathIntake = follower().pathBuilder()
                .addPath(new BezierLine(initialFire, gateIntake))
                .setLinearHeadingInterpolation(initialFire.getHeading(), gateIntake.getHeading())
                .build();


        shootGate = follower().pathBuilder()
                .addPath(new BezierLine(gateIntake, initialFire))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), initialFire.getHeading())
                .build();
    }

    private Command GPP(){
        return new SequentialGroup(
                scorePreload(),
                Intaker.INSTANCE.run,
                new ParallelGroup(
                        new FollowPath(spikeMark1),
                        new SequentialGroup(
                                new Delay(.2),
                                Shooter.INSTANCE.stop
                        )
                ),
                newIntakeSecondSpikeMark(),
                new FollowPath(leverPath).thenWait(0.2),
                scoreSecondSpikeMark(),

                new FollowPath(secondSpikeMarkPath),

                newIntakeFirstSpikeMark(),
                new FollowPath(leverPath2),
                scoreFirstSpikeMark(),
                cycle1(),
                cycle1(),
                Intaker.INSTANCE.stop



        );
    }


    @Override
    public void onInit() {
        //follower = Constants.createFollower(hardwareMap);
        follower().setStartingPose(startPose);
        buildPaths();

    }
    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {


        GPP().schedule();


    }


    public Command turnSpindex(){
        return new InstantCommand(Spindex.INSTANCE::newReTurn);
    }
    public Command spinSpindex(){
        return new InstantCommand(Spindex.INSTANCE::newTurn);
    }
    public Command micro(){
        return new InstantCommand(Spindex.INSTANCE::micro);
    }

    public Command rapid(){
        return new InstantCommand(Spindex.INSTANCE::rapid);
    }

    public Command zero(){
        return new InstantCommand(Spindex.INSTANCE::zeroAuto);
    }
    @Override
    public void onUpdate(){
        follower().update();
    }
}

