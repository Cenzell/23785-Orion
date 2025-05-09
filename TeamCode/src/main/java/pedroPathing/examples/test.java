package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.MotionSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "testing", group = "Examples")
public class test extends OpMode { //TODO This
    private Follower follower;
    private PathChain path;
    private Telemetry telemetryA;

    //private Telemetry telemetry;
    //private HardwareMap hardwareMap;
    //private Gamepad gamepad1;
    //private Gamepad gamepad2;

    private final Pose startPose = new Pose(11, 64, Math.toRadians(0));

    MotionSubsystem motionSubsystem;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motionSubsystem = new MotionSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        motionSubsystem.init();

        motionSubsystem.drivePos();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        PathBuilder builder = new PathBuilder();
        path = builder
                .addPath(new BezierLine(
                        new Point(11.000, 64.000, Point.CARTESIAN),
                        new Point(32.000, 64.000, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Point(32.000, 64.000, Point.CARTESIAN),
                        new Point(16.000, 35.000, Point.CARTESIAN),
                        new Point(60.000, 32.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(60.000, 32.000, Point.CARTESIAN),
                        new Point(60.000, 24.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(60.000, 24.000, Point.CARTESIAN),
                        new Point(22.000, 24.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(22.000, 24.000, Point.CARTESIAN),
                        new Point(60.000, 24.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(60.000, 24.000, Point.CARTESIAN),
                        new Point(60.000, 14.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(60.000, 14.000, Point.CARTESIAN),
                        new Point(22.000, 14.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(22.000, 14.000, Point.CARTESIAN),
                        new Point(60.000, 14.000, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(
                        new Point(60.000, 14.000, Point.CARTESIAN),
                        new Point(60.000, 9.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(60.000, 9.000, Point.CARTESIAN),
                        new Point(22.000, 9.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(22.000, 9.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 62.000, Point.CARTESIAN),
                        new Point(32.000, 62.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(32.000, 62.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 66.000, Point.CARTESIAN),
                        new Point(32.000, 66.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(32.000, 66.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 68.000, Point.CARTESIAN),
                        new Point(32.000, 68.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(32.000, 68.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 70.000, Point.CARTESIAN),
                        new Point(32.000, 70.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(32.000, 70.000, Point.CARTESIAN),
                        new Point(8.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        follower.followPath(path);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(path, true);
        }

        follower.telemetryDebug(telemetryA);
        motionSubsystem.loop();
    }
}