package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subs.Intaker;
import org.firstinspires.ftc.teamcode.subs.Kicker;
import org.firstinspires.ftc.teamcode.subs.Shooter;
import org.firstinspires.ftc.teamcode.subs.Spindex;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Autonomous(name = "close auto")
public class close_auto extends NextFTCOpMode {
    public close_auto() {
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

    public static double posit1 = 0;
    public static double posit2 = 1425.1 * 16/24 * 2/6;
    public static double posit3 = 1425.1 * 16/24 * 4/6;

    public static double posit4 = 1425.1 * 16/24 * 3/6;
    public static double posit5 = 1425.1 * 16/24 * 5/6;
    public static double posit6 = 1425.1 * 16/24 * 1/6;
    private final Pose startPose = new Pose(21.960, 125.225, Math.toRadians(144));

    private final Pose initialFire = new Pose(60.517,81.807,Math.toRadians(135));
    private final Pose spikeMark = new Pose(41.406, 90.154, Math.toRadians(180));
    private final Pose ballMark1 = new Pose(32.210,90.154,Math.toRadians(180));
    private final Pose ballMark2 = new Pose(28.210,90.154,Math.toRadians(180));
    private final Pose ballMark3 = new Pose(24.210,90.154,Math.toRadians(180));



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

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(testPath,true),
                new SequentialGroup(
                        Shooter.INSTANCE.startclose
                ),
                new Delay(1),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        Spindex.INSTANCE.turnIntake(posit2),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
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
                Spindex.INSTANCE.turnIntake(posit5),

                new FollowPath(intake2,true),
                Spindex.INSTANCE.turnIntake(posit6),

                new FollowPath(intake3,true),

                Spindex.INSTANCE.turnIntake(posit1),

                Shooter.INSTANCE.startclose,
                Intaker.INSTANCE.stop,

                new FollowPath(shootMark1),


                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        Spindex.INSTANCE.turnIntake(posit2),
                        new Delay(0.5),
                        Kicker.INSTANCE.toShooter,
                        new Delay(1),
                        Kicker.INSTANCE.toSpindex,
                        new Delay(0.5),
                        Spindex.INSTANCE.turnIntake(posit3),
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





    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        follower().update();
    }
}