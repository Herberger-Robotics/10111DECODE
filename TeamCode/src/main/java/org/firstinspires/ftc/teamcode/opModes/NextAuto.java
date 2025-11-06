package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
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
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.positionable.SetPosition;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;


@Autonomous(name = "autobotsrollout")
public class NextAuto extends NextFTCOpMode {
    public NextAuto() {
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
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    private final Pose initialFire = new Pose(67,16,Math.toRadians(125));
    private final Pose spikeMark = new Pose(40, 36, Math.toRadians(180));



    //private Follower follower;

    private PathChain testPath;
    private PathChain spikeMark1;

    public void buildPaths(){
        testPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, initialFire))
                .setLinearHeadingInterpolation(startPose.getHeading(), initialFire.getHeading())
                .build();
        spikeMark1 = follower().pathBuilder()
                .addPath(new BezierLine(initialFire,spikeMark))
                .setLinearHeadingInterpolation(initialFire.getHeading(),spikeMark.getHeading())
                .build();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(testPath,true),
                new SequentialGroup(
                      Shooter.INSTANCE.start
                ),
                new Delay(1),
                new SequentialGroup(
                        Kicker.INSTANCE.toShooter,
                        Kicker.INSTANCE.toSpindex,
                        Spindex.INSTANCE.turnIntake(posit2),
                        Kicker.INSTANCE.toShooter,
                        Kicker.INSTANCE.toSpindex,
                        Spindex.INSTANCE.turnIntake(posit2),
                        Kicker.INSTANCE.toShooter,
                        Kicker.INSTANCE.toSpindex,
                        Spindex.INSTANCE.turnIntake(posit4)
                ),
                new FollowPath(spikeMark1,true)
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