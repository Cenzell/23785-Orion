package org.firstinspires.ftc.teamcode.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "High-Park", group = "Orion")
public class speci_high_park extends OpMode {
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
                                new Point(10.082, 81.494, Point.CARTESIAN),
                                new Point(33.774, 81.494, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(0,0)
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(33.774, 81.494, Point.CARTESIAN),
                                new Point(22.482, 121.755, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(22.482, 121.755, Point.CARTESIAN),
                                new Point(22.482, 121.755, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(22.482, 121.755, Point.CARTESIAN),
                                new Point(63.303, 117.850, Point.CARTESIAN),
                                new Point(63.658, 99.747, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270));

        follower.setPose(new Pose(10.082, 81.492));

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