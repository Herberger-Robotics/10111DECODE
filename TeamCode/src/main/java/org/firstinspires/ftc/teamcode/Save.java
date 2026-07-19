package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;


public class Save {

    // Robot pose saved at end of Auto, restored at start of TeleOp
    public static Pose savedPose = new Pose(144 - 21.960, 125.225 - 22, Math.toRadians(45));
    public static int side = 0;
    // Add any other cross-OpMode state here as needed
    // e.g. public static Alliance alliance = Alliance.RED;
}