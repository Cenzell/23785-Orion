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
@Autonomous (name = "One-Speci-Park-reid", group = "Orion")
public class one_specimen_reid extends OpMode {
    private Telemetry telemetryA;


    public static double DISTANCE = 40;


    private boolean forward = true;


    private Follower follower;


    private Path forwards;


    @Override
    public void init() {
        follower = new Follower(hardwareMap);


        PathBuilder builder = new PathBuilder();


        // this auto would have 10 points from high specimen + 3 for park in obs zone


        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(10.000, 85.000, Point.CARTESIAN),
                                new Point(10.000, 85.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(10.000, 85.000, Point.CARTESIAN),
                                new Point(40.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(40.000, 70.000, Point.CARTESIAN),
                                new Point(40.000, 110.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(40.000, 110.000, Point.CARTESIAN),
                                new Point(60.000, 110.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(60.000, 110.000, Point.CARTESIAN),
                                new Point(60.000, 110.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(60.000, 110.000, Point.CARTESIAN),
                                new Point(60.000, 98.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));


        follower.setPose(new Pose(10.000, 85.000));


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

