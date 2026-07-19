package org.firstinspires.ftc.teamcode.opModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Save;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subs.Intaker;
import org.firstinspires.ftc.teamcode.subs.Kicker;
import org.firstinspires.ftc.teamcode.subs.Shooter;
import org.firstinspires.ftc.teamcode.subs.Spindex;
import org.firstinspires.ftc.teamcode.subs.TurretSubsystem;

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


@Autonomous(name = "Sorted Red", preselectTeleOp = "Drive")
public class Sorted_Red extends NextFTCOpMode {
    public Sorted_Red() {
        addComponents(
                new SubsystemComponent(
                        Shooter.INSTANCE,
                        Intaker.INSTANCE,
                        Kicker.INSTANCE,
                        Spindex.INSTANCE,
                        TurretSubsystem.INSTANCE
                ),

                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private Limelight3A limelight;

    int pathType = 1;
    private final Pose startPose = new Pose(122.04, 103.225, Math.toRadians(45)); //144 - 21.960, 125.225 - 22

    private final Pose initialFire = new Pose(144 - 49.517,88.807- 25,Math.toRadians(0));
    private final Pose spikeMark = new Pose(144 - 48.406, 88.807 - 46, Math.toRadians(0));

    private final Pose ballMark3 = new Pose(144 - 9,88.807 - 46,Math.toRadians(0));

    private final Pose secondSpikeMarkPos = new Pose(144 - 48.406, 88.807 - 23, Math.toRadians(0));

    private final Pose secondBallMark3 = new Pose(144 - 8.210,88.807 - 23,Math.toRadians(0));

    private final Pose lever = new Pose(144 - 22.210,88.807 - 40,Math.toRadians(0));
    private final Pose backitup = new Pose(144 - 39.210,88.807 - 50,Math.toRadians(0));


    private final Pose thirdSpikeMarkPos = new Pose(95.594, 18.807, Math.toRadians(0)); // 144 - 48.406, 88.807 - 70

    private final Pose thirdBallMark3 = new Pose(138,18.807,Math.toRadians(0)); // 144 - 12, 88.807 - 70



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


    private Command scorePreload(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose,
                new FollowPath(testPath,true),

                rapid().thenWait(.7),
                Shooter.INSTANCE.stop

        );
    };


    private Command newIntakeSecondSpikeMark(){
        return new ParallelGroup(

                new FollowPath(newIntake1, true, 0.7),
                new SequentialGroup(
                        new Delay(0.8),
                        spinSpindex(),
                        new Delay(0.6),
                        spinSpindex(),
                        new Delay(.3)
                )


        );
    };

    private Command scoreSecondSpikeMark(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose.thenWait(0.3),
                micro(),
                new FollowPath(shootMark1,true),
                turnSpindex().thenWait(0.6),
                turnSpindex().thenWait(0.6),
                turnSpindex().thenWait(0.6),
                zero(),
                Shooter.INSTANCE.stop

        );
    };


    private Command newIntakeFirstSpikeMark(){
        return new ParallelGroup(
                new FollowPath(newIntake2, true, 0.7),

                new SequentialGroup(
                        new Delay(0.8),
                        spinSpindex(),
                        new Delay(0.4),
                        spinSpindex(),
                        new Delay(0.3)

                )
        );
    };
    private Command scoreFirstSpikeMark(){
        return new SequentialGroup(



                Shooter.INSTANCE.startclose.thenWait(0.3),
                micro(),
                new FollowPath(secondShootMark1,true),
                turnSpindex().thenWait(0.6),
                turnSpindex().thenWait(0.6),
                turnSpindex().thenWait(0.6),
                zero(),
                Shooter.INSTANCE.stop

        );
    };


    private Command newIntakeThirdSpikeMark(){
        return new SequentialGroup(

                new ParallelGroup(


                new FollowPath(newIntake3, true, 0.7),
                new SequentialGroup(
                        new Delay(0.8),
                        spinSpindex(),
                        new Delay(0.6),
                        spinSpindex(),
                        new Delay(0.4)
                )

            )
        );
    };
    private Command scoreThirdSpikeMark(){
        return new SequentialGroup(

                Shooter.INSTANCE.startclose.thenWait(0.3),
                micro(),
                new FollowPath(thirdShootMark1,true),
                turnSpindex().thenWait(0.6),
                turnSpindex().thenWait(0.6),
                turnSpindex().thenWait(0.6),
                zero(),
                Shooter.INSTANCE.stop

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
                .addPath(new BezierCurve(ballMark3,
                        new Pose(103.79,26.807,Math.toRadians(0))
                        ,initialFire))
                .setLinearHeadingInterpolation(ballMark3.getHeading(), initialFire.getHeading())
                .build();

        leverPath = follower().pathBuilder()
                .addPath(new BezierCurve(secondBallMark3, backitup, lever))
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


        thirdSpikeMarkPath = follower().pathBuilder()
                .addPath(new BezierLine(initialFire,thirdSpikeMarkPos))
                .setLinearHeadingInterpolation(initialFire.getHeading(),thirdSpikeMarkPos.getHeading())
                .build();


        newIntake3 = follower().pathBuilder()
                .addPath(new BezierLine(thirdSpikeMarkPos,thirdBallMark3))
                .setLinearHeadingInterpolation(thirdSpikeMarkPos.getHeading(),ballMark3.getHeading())
                .build();
        thirdShootMark1 = follower().pathBuilder()
                .addPath(new BezierCurve(thirdBallMark3, new Pose(103.79,26.807,Math.toRadians(0)), initialFire))
                .setLinearHeadingInterpolation(thirdBallMark3.getHeading(), initialFire.getHeading())
                .build();

    }

    private Command PPG(){
        return new SequentialGroup(
                scorePreload(),
                Intaker.INSTANCE.run,
                new FollowPath(secondSpikeMarkPath),

                newIntakeFirstSpikeMark(),

                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(0.3),
                                Intaker.INSTANCE.stop
                        ),

                        new FollowPath(leverPath).thenWait(0.2)
                ),
                spinSpindex().thenWait(0.6),

                scoreFirstSpikeMark(),
                new FollowPath(spikeMark1),

                Intaker.INSTANCE.run,

                newIntakeSecondSpikeMark(),
                scoreSecondSpikeMark(),
                new FollowPath(thirdSpikeMarkPath),
                newIntakeThirdSpikeMark(),
                spinSpindex().thenWait(0.6),
                spinSpindex().thenWait(0.6),
                scoreThirdSpikeMark(),
                Intaker.INSTANCE.stop,
                new FollowPath(spikeMark1)


        );
    }

    private Command PGP(){
        return new SequentialGroup(


                scorePreload(),
                Intaker.INSTANCE.run,
                new FollowPath(secondSpikeMarkPath),
                newIntakeFirstSpikeMark(),
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(0.3),
                                Intaker.INSTANCE.stop
                        ),

                        new FollowPath(leverPath).thenWait(0.2)
                ),
                scoreFirstSpikeMark(),
                new FollowPath(spikeMark1),
                Intaker.INSTANCE.run,
                newIntakeSecondSpikeMark(),
                spinSpindex().thenWait(0.5),
                spinSpindex().thenWait(0.5),
                scoreSecondSpikeMark(),
                new FollowPath(thirdSpikeMarkPath),
                newIntakeThirdSpikeMark(),
                spinSpindex().thenWait(0.5),
                scoreThirdSpikeMark(),
                Intaker.INSTANCE.stop,
                new FollowPath(spikeMark1)


        );
    }
    private Command GPP(){
        return new SequentialGroup(


                scorePreload(),
                Intaker.INSTANCE.run,
                new FollowPath(secondSpikeMarkPath),
                newIntakeFirstSpikeMark(),
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(0.3),
                                Intaker.INSTANCE.stop
                        ),

                        new FollowPath(leverPath).thenWait(0.2)
                ),
                spinSpindex().thenWait(0.5),
                spinSpindex().thenWait(0.5),
                scoreFirstSpikeMark(),
                new FollowPath(spikeMark1),

                Intaker.INSTANCE.run,
                newIntakeSecondSpikeMark(),
                spinSpindex().thenWait(0.5),
                scoreSecondSpikeMark(),
                new FollowPath(thirdSpikeMarkPath),
                newIntakeThirdSpikeMark(),
                scoreThirdSpikeMark(),
                Intaker.INSTANCE.stop,
                new FollowPath(spikeMark1)


        );
    }

    @Override
    public void onInit() {
        Save.side = 0;

        follower().setStartingPose(startPose);
        buildPaths();
        initializeLimelight();


    }
    @Override
    public void onWaitForStart() {
        int result = processLimelightResults();

        if(result == 21){
            pathType = 1;
        } else if(result == 22){
            pathType = 2;

        } else if(result == 23){
            pathType = 0;
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
                PGP().schedule();
                break;
            case 2:
                PPG().schedule();
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


    @Override
    public void onStop(){
        Save.savedPose = follower().getPose();
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

