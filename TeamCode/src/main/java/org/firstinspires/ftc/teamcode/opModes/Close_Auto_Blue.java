package org.firstinspires.ftc.teamcode.opModes;

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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.util.List;


@Autonomous(name = "Blue Close Auto", preselectTeleOp = "Drive")
public class Close_Auto_Blue extends NextFTCOpMode {
    public Close_Auto_Blue() {
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
    public static double posit1 = 0;
    public static double posit2 = 1425.1 * 16/24 * 2/6;
    public static double posit3 = 1425.1 * 16/24 * 4/6;

    public static double posit4 = 1425.1 * 16/24 * 3/6;
    public static double posit5 = 1425.1 * 16/24 * 5/6;
    public static double posit6 = 1425.1 * 16/24 * 1/6;
    private final Pose startPose = new Pose(21.960, 125.225, Math.toRadians(0));

    private final Pose initialFire = new Pose(68.517,88.807,Math.toRadians(124));
    private final Pose spikeMark = new Pose(50.406, 103.154, Math.toRadians(180));
    private final Pose ballMark1 = new Pose(46.210,103.154,Math.toRadians(180));
    private final Pose ballMark2 = new Pose(41.210,103.154,Math.toRadians(180));
    private final Pose ballMark3 = new Pose(36.210,103.154,Math.toRadians(180));

    private final Pose secondSpikeMarkPos = new Pose(53.406, 79.154, Math.toRadians(180));
    private final Pose secondBallMark1 = new Pose(47.210,79.154,Math.toRadians(180));
    private final Pose secondBallMark2 = new Pose(41.210,79.154,Math.toRadians(180));
    private final Pose secondBallMark3 = new Pose(36.210,79.154,Math.toRadians(180));


    //private Follower follower;

    private PathChain testPath;
    private PathChain spikeMark1;
    private PathChain intake1;
    private PathChain intake2;
    private PathChain intake3;
    private PathChain shootMark1;

    private PathChain secondSpikeMarkPath;
    private PathChain secondIntake1;
    private PathChain secondIntake2;
    private PathChain secondIntake3;
    private PathChain secondShootMark1;

    //PRELOAD ORDER

    //SLOT 1: PURPLE - shooter:POSIT1 - intake: POSIT4
    //SLOT 2: GREEN - shooter: POSIT2 - intake: POSIT5
    //SLOT 3: PURPLE - shooter: POSIT3 - intake: POSIT6




    public void buildPaths(){
        testPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, initialFire))
                .setLinearHeadingInterpolation(startPose.getHeading(), initialFire.getHeading())
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
        shootMark1 = follower().pathBuilder()
                .addPath(new BezierLine(ballMark3, initialFire))
                .setLinearHeadingInterpolation(ballMark3.getHeading(), initialFire.getHeading())
                .build();


        secondSpikeMarkPath = follower().pathBuilder()
                .addPath(new BezierLine(initialFire,secondSpikeMarkPos))
                .setLinearHeadingInterpolation(initialFire.getHeading(),secondSpikeMarkPos.getHeading())
                .build();

        secondIntake1 = follower().pathBuilder()
                .addPath(new BezierLine(spikeMark,secondBallMark1))
                .setLinearHeadingInterpolation(spikeMark.getHeading(),secondBallMark1.getHeading())
                .build();
        secondIntake2 = follower().pathBuilder()
                .addPath(new BezierLine(ballMark1,secondBallMark2))
                .setLinearHeadingInterpolation(ballMark1.getHeading(),secondBallMark2.getHeading())
                .build();
        secondIntake3 = follower().pathBuilder()
                .addPath(new BezierLine(ballMark2,secondBallMark3))
                .setLinearHeadingInterpolation(ballMark2.getHeading(),secondBallMark3.getHeading())
                .build();
        secondShootMark1 = follower().pathBuilder()
                .addPath(new BezierLine(secondBallMark3, initialFire))
                .setLinearHeadingInterpolation(secondBallMark3.getHeading(), initialFire.getHeading())
                .build();



    }
    private Command PPG() {
        return new SequentialGroup(
                Shooter.INSTANCE.startclose,

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit1),

                new FollowPath(testPath,true),
                new Delay(0.25),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.5),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //GREEN
                        Spindex.INSTANCE.turnIntake(posit2),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop

                ),

                new FollowPath(spikeMark1,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit4),

                Intaker.INSTANCE.run,
                new FollowPath(intake1,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(intake2,true),

                //INTAKING GREEN
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(intake3,true),
                Shooter.INSTANCE.startclose,

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit1),

                Intaker.INSTANCE.stop,

                new FollowPath(shootMark1),

                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //GREEN
                        Spindex.INSTANCE.turnIntake(posit2),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,

                        Shooter.INSTANCE.stop
                ),

                //this is the SECOND spike mark pick up start
                new FollowPath(secondSpikeMarkPath,true),

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit4),

                Intaker.INSTANCE.run,

                new FollowPath(secondIntake1,true),

                //GREEN
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(secondIntake2,true),

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(secondIntake3,true),
                Shooter.INSTANCE.startclose,

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit1),

                Intaker.INSTANCE.stop,

                new FollowPath(secondShootMark1),

                //TODO: CHANGE WHAT POSITION IS CHANGED FOR SPINDEX
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit2),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //GREEN
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop
                )



        );
    }
    private Command PGP() {
        return new SequentialGroup(
                Shooter.INSTANCE.startclose,

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit1),

                new FollowPath(testPath,true),
                new Delay(0.25),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.5),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //GREEN
                        Spindex.INSTANCE.turnIntake(posit2),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop

                ),

                new FollowPath(spikeMark1,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit4),

                Intaker.INSTANCE.run,
                new FollowPath(intake1,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(intake2,true),

                //INTAKING GREEN
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(intake3,true),
                Shooter.INSTANCE.startclose,

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit1),

                Intaker.INSTANCE.stop,

                new FollowPath(shootMark1),

                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //GREEN
                        Spindex.INSTANCE.turnIntake(posit2),

                        new Delay(0.25),
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,

                        Shooter.INSTANCE.stop
                ),

                //this is the SECOND spike mark pick up start
                new FollowPath(secondSpikeMarkPath,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit4),

                Intaker.INSTANCE.run,

                new FollowPath(secondIntake1,true),

                //INTAKING GREEN
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(secondIntake2,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(secondIntake3,true),
                Shooter.INSTANCE.startclose,

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit1),

                Intaker.INSTANCE.stop,

                new FollowPath(secondShootMark1),

                //TODO: CHANGE WHAT POSITION IS CHANGED FOR SPINDEX
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //GREEN
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit2),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop
                )



        );
    }
    private Command GPP() {
        return new SequentialGroup(
                Shooter.INSTANCE.startclose,

                //GREEN
                Spindex.INSTANCE.turnIntake(posit2),

                new FollowPath(testPath,true),
                new Delay(0.25),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.5),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit1),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop

                ),

                new FollowPath(spikeMark1,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit4),

                Intaker.INSTANCE.run,
                new FollowPath(intake1,true),

                //INTAKING PURPLE
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(intake2,true),

                //INTAKING GREEN
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(intake3,true),
                Shooter.INSTANCE.startclose,

                //GREEN
                Spindex.INSTANCE.turnIntake(posit2),

                Intaker.INSTANCE.stop,

                new FollowPath(shootMark1),

                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit1),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,

                        Shooter.INSTANCE.stop
                ),

                //this is the SECOND spike mark pick up start
                new FollowPath(secondSpikeMarkPath,true),

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit4),

                Intaker.INSTANCE.run,

                new FollowPath(secondIntake1,true),

                //GREEN
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(secondIntake2,true),

                //PURPLE
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(secondIntake3,true),
                Shooter.INSTANCE.startclose,

                //GREEN
                Spindex.INSTANCE.turnIntake(posit1),

                Intaker.INSTANCE.stop,

                new FollowPath(secondShootMark1),

                //TODO: CHANGE WHAT POSITION IS CHANGED FOR SPINDEX
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit2),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.25),

                        //PURPLE
                        Spindex.INSTANCE.turnIntake(posit3),

                        new Delay(0.25),

                        Kicker.INSTANCE.toShooter,
                        new Delay(0.25),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop
                )



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
                PGP().schedule();
                break;
            case 2:
                PPG().schedule();
                break;
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
    public void onUpdate(){
        follower().update();
    }
}

