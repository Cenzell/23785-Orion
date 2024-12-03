package org.firstinspires.ftc.teamcode.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.follower.Follower;

import org.firstinspires.ftc.teamcode.lib.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Speci-Park", group = "Orion")
public class test2 extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;

    PathBuilder builder = new PathBuilder();

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.757, 84.983, Point.CARTESIAN),
                                new Point(36.668, 84.983, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
    }

    @Override
    public void loop() {
        follower.setPose(new Pose(28.892, 90.838));

        follower.followPath(builder.build());

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();

        follower.update();

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
}