package org.firstinspires.ftc.teamcode.BigArm.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Left Park", group = "Orion")
public class left_park extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(11.095, 56.629, Point.CARTESIAN),
                                new Point(11.788, 10.401, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));

        follower.setPose(new Pose(11.095, 56.629));

        follower.followPath(builder.build());

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}