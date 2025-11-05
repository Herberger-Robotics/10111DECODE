package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "speed i need this")
public class Testing_Auto extends NextFTCOpMode {
    public Testing_Auto() {
        addComponents(
                new SubsystemComponent(),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private final Pose startPose = new Pose(87.842, 8.214, Math.toRadians(90));

    private final Pose initialFire = new Pose(87.842,18.775,Math.toRadians(125));
    private final Pose spikeMark = new Pose(103.097, 35.539, 0);

    private Follower follower;

    private PathChain testPath;
    private PathChain spikeMark1;

    public void buildPaths(){
        testPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, initialFire))
                .setLinearHeadingInterpolation(startPose.getHeading(), initialFire.getHeading())
                .build();
        spikeMark1 = follower.pathBuilder()
                .addPath(new BezierLine(initialFire,spikeMark))
                .setLinearHeadingInterpolation(initialFire.getHeading(),spikeMark.getHeading())
                .build();
     }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(testPath,true),
                new FollowPath(spikeMark1,true)
        );
    }
    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);




    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        follower.update();
    }
}