package org.firstinspires.ftc.teamcode.opModes;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "Blue Far Auto")
public class Far_Auto_Red extends NextFTCOpMode {
    public Far_Auto_Red() {
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
    private final Pose startPose = new Pose(56.661, 8.047, Math.toRadians(90)).mirror();

    private final Pose initialFire = new Pose(57.835,17.434,Math.toRadians(114)).mirror();

    private final Pose spikeMark = new Pose(50.406, 35, Math.toRadians(180)).mirror();
    private final Pose ballMark1 = new Pose(46.210,35,Math.toRadians(180)).mirror();
    private final Pose ballMark2 = new Pose(42.210,35,Math.toRadians(180)).mirror();
    private final Pose ballMark3 = new Pose(36.210,35,Math.toRadians(180)).mirror();



    //private Follower follower;


    private PathChain testPath;
    private PathChain spikeMark1;
    private PathChain intake1;
    private PathChain intake2;
    private PathChain intake3;
    private PathChain shootMark1;
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

    }

    private Command PPG() {
        return new SequentialGroup(
                //must be purple
                Shooter.INSTANCE.startclose,
                Spindex.INSTANCE.turnIntake(posit1),
                new FollowPath(testPath,true),
                new Delay(1),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        //should be purple
                        Spindex.INSTANCE.turnIntake(posit3),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        //should be green
                        Spindex.INSTANCE.turnIntake(posit2),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop,
                        new Delay(0.5),

                        Spindex.INSTANCE.turnIntake(posit4)
                ),
                new FollowPath(spikeMark1,true),

                Intaker.INSTANCE.run,

                new FollowPath(intake1,true),
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(intake2,true),
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(intake3,true),

                //should be purple
                Spindex.INSTANCE.turnIntake(posit1),

                Shooter.INSTANCE.startclose,
                Intaker.INSTANCE.stop,

                new FollowPath(shootMark1),

                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),

                        //should be purple
                        Spindex.INSTANCE.turnIntake(posit3),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),

                        //should be green
                        Spindex.INSTANCE.turnIntake(posit2),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop
                )


        );
    }
    private Command PGP() {
        return new SequentialGroup(
                //must be purple
                Shooter.INSTANCE.startclose,
                Spindex.INSTANCE.turnIntake(posit1),
                new FollowPath(testPath,true),

                new Delay(1),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        //should be purple
                        Spindex.INSTANCE.turnIntake(posit2),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        //should be green
                        Spindex.INSTANCE.turnIntake(posit3),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop,
                        new Delay(0.5),
                        Spindex.INSTANCE.turnIntake(posit4)
                ),
                new FollowPath(spikeMark1,true),

                Intaker.INSTANCE.run,

                new FollowPath(intake1,true),
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(intake2,true),
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(intake3,true),

                //should be purple
                Spindex.INSTANCE.turnIntake(posit1),

                Shooter.INSTANCE.startclose,
                Intaker.INSTANCE.stop,

                new FollowPath(shootMark1),

                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),

                        //should be purple
                        Spindex.INSTANCE.turnIntake(posit2),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),

                        //should be green
                        Spindex.INSTANCE.turnIntake(posit3),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop
                )


        );
    }
    private Command GPP() {
        return new SequentialGroup(
                //must be purple
                Shooter.INSTANCE.startclose,
                Spindex.INSTANCE.turnIntake(posit2),
                new FollowPath(testPath,true),

                new Delay(1),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        //should be purple
                        Spindex.INSTANCE.turnIntake(posit3),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        //should be green
                        Spindex.INSTANCE.turnIntake(posit1),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        Shooter.INSTANCE.stop,
                        new Delay(0.5),

                        Spindex.INSTANCE.turnIntake(posit5)
                ),
                new FollowPath(spikeMark1,true),

                Intaker.INSTANCE.run,

                new FollowPath(intake1,true),
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(intake2,true),
                Spindex.INSTANCE.turnIntake(posit4),

                new FollowPath(intake3,true),

                //should be purple
                Spindex.INSTANCE.turnIntake(posit1),

                Shooter.INSTANCE.startclose,
                Intaker.INSTANCE.stop,

                new FollowPath(shootMark1),

                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),

                        //should be purple
                        Spindex.INSTANCE.turnIntake(posit3),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),

                        //should be green
                        Spindex.INSTANCE.turnIntake(posit2),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
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