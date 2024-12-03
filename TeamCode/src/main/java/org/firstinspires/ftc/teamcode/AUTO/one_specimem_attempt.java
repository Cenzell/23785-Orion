package org.firstinspires.ftc.teamcode.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.lib.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "Speci-Park", group = "Orion")
public class one_specimem_attempt extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 90;

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
                                new Point(10.082, 81.492, Point.CARTESIAN),
                                new Point(28.892, 90.838, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(28.892, 90.838, Point.CARTESIAN),
                                new Point(28.892, 105.400, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(28.892, 105.400, Point.CARTESIAN),
                                new Point(63.795, 108.867, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(63.795, 108.867, Point.CARTESIAN),
                                new Point(64.257, 117.650, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(64.257, 117.650, Point.CARTESIAN),
                                new Point(19.185, 116.263, Point.CARTESIAN),
                                new Point(16.642, 127.127, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-4), Math.toRadians(-36))
                .setReversed(true)
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(16.642, 127.127, Point.CARTESIAN),
                                new Point(64.950, 105.631, Point.CARTESIAN),
                                new Point(63.101, 126.896, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(63.101, 126.896, Point.CARTESIAN),
                                new Point(12.944, 127.589, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(12.944, 127.589, Point.CARTESIAN),
                                new Point(70.266, 124.815, Point.CARTESIAN),
                                new Point(64.488, 96.848, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));
    }

    @Override
    public void loop() {
        follower.setPose(new Pose(10.082, 81.492));

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