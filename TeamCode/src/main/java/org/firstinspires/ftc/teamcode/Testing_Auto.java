package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class Testing_Auto extends NextFTCOpMode {
    public Testing_Auto() {
        addComponents(
                new SubsystemComponent(),
                BulkReadComponent.INSTANCE
        );
    }
    private final Pose startPose = new Pose(8.2, 55, Math.toRadians(0.0));

    private Follower follower;

    private PathChain testPath;

    public void buildPaths(){
        testPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(

                new ParallelGroup(
                       // Lift.INSTANCE.toMiddle,
                       // Claw.INSTANCE.close
                ),
                new Delay(0.5),
                new ParallelGroup(
                      //  Claw.INSTANCE.open,
                       // Lift.INSTANCE.toLow
                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}