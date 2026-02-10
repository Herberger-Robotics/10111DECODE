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
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "Rapid Red Close Auto", preselectTeleOp = "Drive")
public class Close_Auto_Red_Rapid extends NextFTCOpMode {
    public Close_Auto_Red_Rapid() {
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
    private final Pose startPose = new Pose(144 - 21.960, 125.225 - 22, Math.toRadians(45));

    private final Pose initialFire = new Pose(144 - 50.517,88.807 - 32,Math.toRadians(52));
    private final Pose spikeMark = new Pose(144 - 48.406, 88.807 - 45, Math.toRadians(0));
    private final Pose ballMark1 = new Pose(144 - 44.210, 88.807 - 45,Math.toRadians(0));
    private final Pose ballMark2 = new Pose(144 - 39.210,88.807 - 45,Math.toRadians(0));
    private final Pose ballMark3 = new Pose(144 - 17.0,88.807 - 45,Math.toRadians(0));

    private final Pose secondSpikeMarkPos = new Pose(144 - 48.406, 88.807 - 20, Math.toRadians(0));
    private final Pose secondBallMark1 = new Pose(144 - 44.210,88.807 - 37,Math.toRadians(0));
    private final Pose secondBallMark2 = new Pose(144 - 39.210,88.807 - 37,Math.toRadians(0));
    private final Pose secondBallMark3 = new Pose(144 - 24.210,88.807 - 20,Math.toRadians(0));

    private final Pose lever = new Pose(144 - 20.210,88.807 - 30,Math.toRadians(0));
    private final Pose backitup = new Pose(144 - 39.210,88.807 - 30,Math.toRadians(0));


    private final Pose thirdSpikeMarkPos = new Pose(144 - 48.406, 88.807 - 70, Math.toRadians(0));
    private final Pose thirdBallMark1 = new Pose(144 - 44.210,88.807 - 70,Math.toRadians(0));
    private final Pose thirdBallMark2 = new Pose(144 - 39.210,88.807 - 70,Math.toRadians(0));
    private final Pose thirdBallMark3 = new Pose(144 - 12,88.807 - 70,Math.toRadians(0));

    private final Pose gateIntake = new Pose(144 - 10,88.807 - 46.5,Math.toRadians(33.5));



    //private Follower follower;


    private PathChain testPath;
    private PathChain spikeMark1;
    private PathChain intake1;
    private PathChain intake2;
    private PathChain intake3;

    private PathChain newIntake1;
    private PathChain shootMark1;

    private PathChain secondSpikeMarkPath;
    private PathChain secondIntake1;
    private PathChain secondIntake2;
    private PathChain secondIntake3;

    private PathChain newIntake2;
    private PathChain secondShootMark1;

    private PathChain thirdSpikeMarkPath;
    private PathChain thirdIntake1;
    private PathChain thirdIntake2;
    private PathChain thirdIntake3;

    private PathChain newIntake3;
    private PathChain thirdShootMark1;

    private PathChain leverPath;

    private PathChain leverPathIntake;

    private PathChain shootGate;
    private Command scorePreload(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose,
                new FollowPath(testPath,true),

                rapid().thenWait(.6),
                Shooter.INSTANCE.stop

        );
    };
    private Command intakeSecondSpikeMark(){
        return new SequentialGroup(
                new FollowPath(spikeMark1,true),
                Intaker.INSTANCE.run,
                new FollowPath(intake1,true),
                spinSpindex(),
                new FollowPath(intake2,true),
                spinSpindex(),
                new FollowPath(intake3,true),
                Intaker.INSTANCE.stop


        );
    };

    private Command newIntakeSecondSpikeMark(){
        return new ParallelGroup(

                new FollowPath(newIntake1, true, 0.7),
                new SequentialGroup(
                        new Delay(0.6),
                        spinSpindex(),
                        new Delay(0.4),
                        spinSpindex(),
                        new Delay(.3)
                )


        );
    };

    private Command scoreSecondSpikeMark(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose,
                new FollowPath(shootMark1,true),

                rapid().thenWait(.6),
                Shooter.INSTANCE.stop

        );
    };
    private Command intakeFirstSpikeMark(){
        return new SequentialGroup(
                new FollowPath(secondSpikeMarkPath,true),
                Intaker.INSTANCE.run,
                new FollowPath(secondIntake1,true),
                spinSpindex(),
                new FollowPath(secondIntake2,true),
                spinSpindex(),
                new FollowPath(secondIntake3,true),
                Intaker.INSTANCE.stop


        );
    };

    private Command newIntakeFirstSpikeMark(){
        return new ParallelGroup(
                new FollowPath(newIntake2, true, 0.7),

                new SequentialGroup(
                        new Delay(0.6),
                        spinSpindex(),
                        new Delay(0.4),
                        spinSpindex(),
                        new Delay(0.3)

                )
        );
    };
    private Command scoreFirstSpikeMark(){
        return new SequentialGroup(



                Shooter.INSTANCE.startclose,
                new FollowPath(secondShootMark1,true),

                rapid().thenWait(.6),
                Shooter.INSTANCE.stop

        );
    };

    private Command intakeThirdSpikeMark(){
        return new SequentialGroup(
                new FollowPath(thirdSpikeMarkPath,true),
                Intaker.INSTANCE.run,
                new FollowPath(thirdIntake1,true),
                spinSpindex(),
                new FollowPath(thirdIntake2,true),
                spinSpindex(),
                new FollowPath(thirdIntake3,true),
                Intaker.INSTANCE.stop


        );
    };

    private Command newIntakeThirdSpikeMark(){
        return new SequentialGroup(

                new ParallelGroup(


                new FollowPath(newIntake3, true, 0.7),
                new SequentialGroup(
                        new Delay(0.8),
                        spinSpindex(),
                        new Delay(0.4),
                        spinSpindex(),
                        new Delay(0.3)
                )

            )
        );
    };
    private Command scoreThirdSpikeMark(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose,
                new FollowPath(thirdShootMark1,true),

                rapid().thenWait(0.6),
                Shooter.INSTANCE.stop

        );
    };


    private Command cycle2(){
        return new SequentialGroup(
                new FollowPath(leverPathIntake,true),
                new SequentialGroup(
                        new Delay(1.1),
                        spinSpindex(),
                        new Delay(0.3),

                        Shooter.INSTANCE.startclose,
                        new ParallelGroup(
                                new FollowPath(shootGate,true),
                                new SequentialGroup(
                                        new Delay(0.4),
                                        spinSpindex().thenWait(0.2)
                                )
                                ),
                        rapid().thenWait(0.6),
                        Shooter.INSTANCE.stop
                )


        );
    };

    private Command cycle1(){
        return new SequentialGroup(
                new FollowPath(leverPathIntake,true),
                new SequentialGroup(
                        new Delay(1.1),
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
                        rapid().thenWait(0.6),
                        Shooter.INSTANCE.stop
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
        intake1 = follower().pathBuilder()
                .addPath(new BezierLine(spikeMark,ballMark1))
                .setLinearHeadingInterpolation(spikeMark.getHeading(),ballMark1.getHeading())
                .build();
        intake2 = follower().pathBuilder()
                .addPath(new BezierLine(ballMark1,ballMark2))
                .setLinearHeadingInterpolation(ballMark1.getHeading(),ballMark2.getHeading())
                .build();
        intake3 = follower().pathBuilder()
                .addPath(new BezierLine(ballMark2,ballMark3))
                .setLinearHeadingInterpolation(ballMark2.getHeading(),ballMark3.getHeading())
                .build();



        newIntake1 = follower().pathBuilder()
                .addPath(new BezierLine(spikeMark,ballMark3))
                .setLinearHeadingInterpolation(ballMark2.getHeading(),ballMark3.getHeading())
                .build();



        shootMark1 = follower().pathBuilder()
                .addPath(new BezierCurve(ballMark3,
                        new Pose(144 - 40.210,88.807 - 62,Math.toRadians(0))
                        ,initialFire))
                .setLinearHeadingInterpolation(ballMark3.getHeading(), initialFire.getHeading())
                .build();

        leverPath = follower().pathBuilder()
                .addPath(new BezierCurve(secondBallMark3, backitup, lever))
                .setLinearHeadingInterpolation(secondBallMark3.getHeading(), lever.getHeading())
                .build();
        secondSpikeMarkPath = follower().pathBuilder()
                .addPath(new BezierLine(initialFire,secondSpikeMarkPos))
                .setLinearHeadingInterpolation(initialFire.getHeading(), secondBallMark2.getHeading())
                .build();

        secondIntake1 = follower().pathBuilder()
                .addPath(new BezierLine(secondSpikeMarkPos,secondBallMark1))
                .setLinearHeadingInterpolation(secondSpikeMarkPos.getHeading(),secondBallMark1.getHeading())
                .build();
        secondIntake2 = follower().pathBuilder()
                .addPath(new BezierLine(secondBallMark1,secondBallMark2))
                .setLinearHeadingInterpolation(secondBallMark1.getHeading(),secondBallMark2.getHeading())
                .build();
        secondIntake3 = follower().pathBuilder()
                .addPath(new BezierLine(secondBallMark2,secondBallMark3))
                .setLinearHeadingInterpolation(secondBallMark2.getHeading(),secondBallMark3.getHeading())
                .build();


        newIntake2 = follower().pathBuilder()
                .addPath(new BezierLine(secondSpikeMarkPos,secondBallMark3))
                .setLinearHeadingInterpolation(ballMark2.getHeading(),ballMark3.getHeading())
                .build();


        secondShootMark1 = follower().pathBuilder()
                .addPath(new BezierLine(lever,initialFire))
                .setLinearHeadingInterpolation(lever.getHeading(), initialFire.getHeading())
                .build();


        thirdSpikeMarkPath = follower().pathBuilder()
                .addPath(new BezierLine(initialFire,thirdSpikeMarkPos))
                .setLinearHeadingInterpolation(initialFire.getHeading(),thirdSpikeMarkPos.getHeading())
                .build();

        thirdIntake1 = follower().pathBuilder()
                .addPath(new BezierLine(secondSpikeMarkPos,thirdBallMark1))
                .setLinearHeadingInterpolation(thirdSpikeMarkPos.getHeading(),thirdBallMark1.getHeading())
                .build();
        thirdIntake2 = follower().pathBuilder()
                .addPath(new BezierLine(thirdBallMark1,thirdBallMark2))
                .setLinearHeadingInterpolation(thirdBallMark1.getHeading(),thirdBallMark2.getHeading())
                .build();
        thirdIntake3 = follower().pathBuilder()
                .addPath(new BezierLine(thirdBallMark2,thirdBallMark3))
                .setLinearHeadingInterpolation(thirdBallMark2.getHeading(),thirdBallMark3.getHeading())
                .build();
        newIntake3 = follower().pathBuilder()
                .addPath(new BezierLine(thirdSpikeMarkPos,thirdBallMark3))
                .setLinearHeadingInterpolation(ballMark2.getHeading(),ballMark3.getHeading())
                .build();
        thirdShootMark1 = follower().pathBuilder()
                .addPath(new BezierCurve(thirdBallMark3, new Pose(144 - 40.210,88.807 - 62,Math.toRadians(0)), initialFire))
                .setLinearHeadingInterpolation(thirdBallMark3.getHeading(), initialFire.getHeading())
                .build();

        leverPathIntake = follower().pathBuilder()
                .addPath(new BezierCurve(initialFire, new Pose(144 - 40,88.807 - 40,Math.toRadians(35)), gateIntake))
                .setLinearHeadingInterpolation(initialFire.getHeading(), gateIntake.getHeading())
                .build();

        shootGate = follower().pathBuilder()
                .addPath(new BezierCurve(gateIntake, new Pose(144 - 40,88.807 - 40,Math.toRadians(35)),initialFire))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), initialFire.getHeading())
                .build();
    }

    private Command GPP(){
        return new SequentialGroup(
                scorePreload(),
                Intaker.INSTANCE.run,
                new FollowPath(spikeMark1),
                newIntakeSecondSpikeMark(),
                scoreSecondSpikeMark(),
                cycle1(),
                cycle2(),
                new FollowPath(secondSpikeMarkPath),
                newIntakeFirstSpikeMark(),
                scoreFirstSpikeMark(),
                new FollowPath(thirdSpikeMarkPath),
                newIntakeThirdSpikeMark(),
                scoreThirdSpikeMark(),
                Intaker.INSTANCE.stop

                /*scorePreload(),
                Intaker.INSTANCE.run,
                new FollowPath(secondSpikeMarkPath),
                newIntakeFirstSpikeMark(),
                new FollowPath(leverPath),
                scoreFirstSpikeMark(),
                new FollowPath(spikeMark1),
                newIntakeSecondSpikeMark(),
                scoreSecondSpikeMark(),
                Intaker.INSTANCE.run,
                new FollowPath(thirdSpikeMarkPath),
                newIntakeThirdSpikeMark(),
                scoreThirdSpikeMark()*/


        );
    }

    private Command PGP(){
        return new SequentialGroup(


                scorePreload(),
                Intaker.INSTANCE.run,
                new FollowPath(secondSpikeMarkPath),
                newIntakeFirstSpikeMark(),
                new FollowPath(leverPath),
                scoreFirstSpikeMark(),
                new FollowPath(spikeMark1),
                newIntakeSecondSpikeMark(),
                scoreSecondSpikeMark(),
                new FollowPath(thirdSpikeMarkPath),
                newIntakeThirdSpikeMark(),
                scoreThirdSpikeMark()

//                new FollowPath(spikeMark1),
//
//                Intaker.INSTANCE.run,
//                newIntakeSecondSpikeMark(),
//                Intaker.INSTANCE.stop,


             //   scoreSecondSpikeMark(),
//                new FollowPath(thirdSpikeMarkPath),
//
//                Intaker.INSTANCE.run,
//                newIntakeThirdSpikeMark(),
//                Intaker.INSTANCE.stop,
//                scoreThirdSpikeMark(),


        );
    }
    private Command PPG(){
        return new SequentialGroup(


                scorePreload(),
                Intaker.INSTANCE.run,
                new FollowPath(secondSpikeMarkPath),
                
                newIntakeFirstSpikeMark(),
                new FollowPath(leverPath),
                scoreFirstSpikeMark(),
                new FollowPath(spikeMark1),
                newIntakeSecondSpikeMark(),
                scoreSecondSpikeMark(),
                new FollowPath(thirdSpikeMarkPath),
                newIntakeThirdSpikeMark(),
                scoreThirdSpikeMark()


//
//               new FollowPath(spikeMark1),
//
//                Intaker.INSTANCE.run,
//                newIntakeSecondSpikeMark(),
//                Intaker.INSTANCE.stop,

               // scoreSecondSpikeMark(),
//                new FollowPath(thirdSpikeMarkPath),
//
//                Intaker.INSTANCE.run,
//                newIntakeThirdSpikeMark(),
//                Intaker.INSTANCE.stop,
//                scoreThirdSpikeMark(),


        );
    }

    @Override
    public void onInit() {
        //follower = Constants.createFollower(hardwareMap);
        follower().setStartingPose(startPose);
        buildPaths();
        initializeLimelight();

    }
    @Override
    public void onWaitForStart() {
        //follower = Constants.createFollower(hardwareMap);
        int result = processLimelightResults();

        if(result == 21){
            pathType = 0;
        } else if(result == 22){
            pathType = 1;

        } else if(result == 23){
            pathType = 2;
        }else{
            pathType = 0;
        }
        telemetry.addData("Path type", pathType);
        telemetry.update();

    }

    @Override
    public void onStartButtonPressed() {
        switch(pathType){
            case 0:
                GPP().schedule();
                break;
            case 1:
                GPP().schedule();
                break;
            case 2:
                GPP().schedule();
        }

        stopLimelight();

    }

    public Command stopLimelight(){
        return new InstantCommand(()->{
            limelight.stop();
        });
    }
    private int processLimelightResults() {
        List<LLResultTypes.FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        int id = 0;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
        }
        return id;
    }
    private void initializeLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

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

