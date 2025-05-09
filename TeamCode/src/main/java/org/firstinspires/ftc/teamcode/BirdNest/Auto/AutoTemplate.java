package org.firstinspires.ftc.teamcode.BirdNest.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.MotionSubsystem;

@Autonomous(name = "") //TODO: Add name here
public class AutoTemplate extends OpMode {
    Follower follower;
    MotionSubsystem motionSubsystem;

    @Override
    public void init() {
        Constants constants = new Constants();
        follower = new Follower(hardwareMap);
        follower.setPose(new Pose(0,0)); //TODO: Start pose here

        motionSubsystem.init();
    }

    private void buildAllPaths(){
        //public class GeneratedPath {


            /*
            public GeneratedPath() {
                PathBuilder builder = new PathBuilder();

                builder
                        .addPath(
                                // Line 1
                                new BezierLine(
                                        new Point(11.000, 64.000, Point.CARTESIAN),
                                        new Point(38.000, 64.000, Point.CARTESIAN)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .addPath(
                                // Line 2
                                new BezierCurve(
                                        new Point(38.000, 64.000, Point.CARTESIAN),
                                        new Point(16.000, 35.000, Point.CARTESIAN),
                                        new Point(60.000, 32.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(60.000, 32.000, Point.CARTESIAN),
                                        new Point(60.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(60.000, 24.000, Point.CARTESIAN),
                                        new Point(22.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(22.000, 24.000, Point.CARTESIAN),
                                        new Point(60.000, 24.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(60.000, 24.000, Point.CARTESIAN),
                                        new Point(60.000, 14.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(60.000, 14.000, Point.CARTESIAN),
                                        new Point(22.000, 14.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 8
                                new BezierLine(
                                        new Point(22.000, 14.000, Point.CARTESIAN),
                                        new Point(60.000, 14.000, Point.CARTESIAN)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .addPath(
                                // Line 9
                                new BezierLine(
                                        new Point(60.000, 14.000, Point.CARTESIAN),
                                        new Point(60.000, 9.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 10
                                new BezierLine(
                                        new Point(60.000, 9.000, Point.CARTESIAN),
                                        new Point(22.000, 9.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 11
                                new BezierLine(
                                        new Point(22.000, 9.000, Point.CARTESIAN),
                                        new Point(12.000, 35.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 12
                                new BezierCurve(
                                        new Point(12.000, 35.000, Point.CARTESIAN),
                                        new Point(14.000, 62.000, Point.CARTESIAN),
                                        new Point(38.000, 62.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 13
                                new BezierLine(
                                        new Point(38.000, 62.000, Point.CARTESIAN),
                                        new Point(12.000, 35.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 14
                                new BezierCurve(
                                        new Point(12.000, 35.000, Point.CARTESIAN),
                                        new Point(14.000, 66.000, Point.CARTESIAN),
                                        new Point(38.000, 66.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 15
                                new BezierLine(
                                        new Point(38.000, 66.000, Point.CARTESIAN),
                                        new Point(12.000, 35.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 16
                                new BezierCurve(
                                        new Point(12.000, 35.000, Point.CARTESIAN),
                                        new Point(14.000, 68.000, Point.CARTESIAN),
                                        new Point(38.000, 68.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 17
                                new BezierLine(
                                        new Point(38.000, 68.000, Point.CARTESIAN),
                                        new Point(12.000, 35.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 18
                                new BezierCurve(
                                        new Point(12.000, 35.000, Point.CARTESIAN),
                                        new Point(14.000, 70.000, Point.CARTESIAN),
                                        new Point(38.000, 70.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .addPath(
                                // Line 19
                                new BezierLine(
                                        new Point(38.000, 70.000, Point.CARTESIAN),
                                        new Point(8.000, 28.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
            }
        }
        */
        //PathBuilder scoreOne = newPathBuilder();
    }

    @Override
    public void loop() {
        motionSubsystem.loop();


    }
}
