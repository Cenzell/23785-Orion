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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.MotionSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Test Number Too Many")
public class Auto extends OpMode {
    private Follower follower;
    private List<PathChain> paths = new ArrayList<>();
    private Telemetry telemetryA;
    private final Pose startPose = new Pose(11, 64, Math.toRadians(180));

    MotionSubsystem motionSubsystem;
    private int currentStage = 1;

    private ElapsedTime Timer = new ElapsedTime();
    private boolean TimerStarted = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motionSubsystem = new MotionSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        motionSubsystem.init();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        initializePaths();
        follower.setMaxPower(0);
        follower.followPath(paths.get(0));

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void initializePaths() {
        PathBuilder builder;

        // specimen lineup
        //0
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(11.000, 64.000, Point.CARTESIAN),
                        new Point(35.5000, 64.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).setPathEndVelocityConstraint(0)
                .build());

        // push prep 0
        //1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(35.500, 64.000, Point.CARTESIAN),
                        new Point(16.000, 48.000, Point.CARTESIAN),
                        new Point(60.000, 30.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push prep 1
        //2
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 30.000, Point.CARTESIAN),
                        new Point(60.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push 1
        //3
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 28.000, Point.CARTESIAN),
                        new Point(26.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // return 1
        //4
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(26.000, 25.000, Point.CARTESIAN),
                        new Point(60.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push prep
        // 5
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 29.000, Point.CARTESIAN),
                        new Point(60.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push 2
        //6
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 20.000, Point.CARTESIAN),
                        new Point(26.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        // return 2
        //7
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(26.000, 20.000, Point.CARTESIAN),
                        new Point(62.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        //push prep 3
        //8
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(62.000, 20.000, Point.CARTESIAN),
                        new Point(62.000, 12.500, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //push 3
        //9
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(62.000, 11.000, Point.CARTESIAN),
                        new Point(25.000, 11.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //speci 2
        //10
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(25.000, 11.000, Point.CARTESIAN),
                        new Point(32.000, 37.000, Point.CARTESIAN),
                        new Point(12.000, 31.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //score 2
        //11
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(12.000, 31.000, Point.CARTESIAN),
                        new Point(14.000, 62.000, Point.CARTESIAN),
                        new Point(32.000, 62.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // speci 3
        //12
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(32.000, 62.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //score 3
        //13
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 66.000, Point.CARTESIAN),
                        new Point(32.000, 66.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // speci 4
        //14
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(32, 66, Point.CARTESIAN),
                        //new Point(14.000, 70.000, Point.CARTESIAN),
                        new Point(12, 35, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //score 4
        //15
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 66.000, Point.CARTESIAN),
                        new Point(32.000, 66.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //park I hope
        //16
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(8.000, 28.000, Point.CARTESIAN),
                        new Point(8.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
    }

    private boolean isMotionComplete() {
        boolean vertComplete = motionSubsystem.vertExtension.atSetPoint();
        boolean miniArmComplete = motionSubsystem.miniArmPID.atSetPoint();

        return  vertComplete && miniArmComplete;
    }

    public void setPath(int path){
        if(follower.getCurrentPath() != paths.get(path).getPath(path)){
            follower.followPath(paths.get(path));
        }
    }

    @Override
    public void loop() {
        follower.update();
        motionSubsystem.loop();
        follower.setMaxPower(1);

        switch (currentStage) {
            case 1: //lineup to score specimen
                motionSubsystem.specimenPrep();
                motionSubsystem.closeClaw();
                if (follower.atParametricEnd()) {
                    currentStage = 2;
                }
                break;

            case 2: //score specimen and hold location
                motionSubsystem.specimenScore();
                motionSubsystem.setVertTarget(5.8);

                //follower.setMaxPower(1);
                follower.holdPoint(new Pose(40.5000, 64.000, Math.toRadians(180)));
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 3;
                    follower.followPath(paths.get(1));
                }
                break;

            case 3: // move to next position
                motionSubsystem.clawOpen();
                motionSubsystem.drivePos();
                //follower.setMaxPower(1);
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 4;
                    follower.followPath(paths.get(2));
                }
                break;

            case 4: //move to align with push 1
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 5;
                    follower.followPath(paths.get(3));
                }
                break;

            case 5: //push piece one
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 6;
                    follower.followPath(paths.get(4));
                }
                break;

            case 6: //return push 1
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 7;
                    follower.followPath(paths.get(5));
                }
                break;

            case 7: //allign for push 2
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 8;
                    follower.followPath(paths.get(6));
                }
                break;

            case 8: //push piece 2
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 9;
                    follower.followPath(paths.get(7));
                }
                break;

            case 9: //return piece 2
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 10;
                    follower.followPath(paths.get(8));
                }
                break;

            case 10: //align piece 3
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 11;
                    follower.followPath(paths.get(9));
                }
                break;

            case 11: //push piece 3
                motionSubsystem.wallPickupPrep();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 12;
                    follower.followPath(paths.get(10));
                }
                break;

            case 12://align for pickup
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 13;
                }
                break;

            case 13://close claw
                motionSubsystem.closeClaw();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 14;
                }

            case 14://pull off wall
                motionSubsystem.specimenPrep();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 15;
                    follower.followPath(paths.get(11));
                }
                break;

            case 15://drive to score 2
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 16;
                }
                break;

            case 16://score 2
                motionSubsystem.specimenScore();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 17;
                    follower.followPath(paths.get(12));
                }
                break;

            case 17://align for pickup 3
                motionSubsystem.clawOpen();
                motionSubsystem.wallPickupPrep();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 18;
                }
                break;

            case 18://grab from wall
                motionSubsystem.closeClaw();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 19;
                }
                break;

            case 19://pull off wall 3
                motionSubsystem.specimenPrep();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 20;
                    follower.followPath(paths.get(13));
                }
                break;

            case 20://drive to score 3
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 21;
                }
                break;

            case 21:
                motionSubsystem.specimenScore();
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 22;
                    follower.followPath(paths.get(16));
                }
                break;

            case 22:
                if (follower.atParametricEnd() && isMotionComplete()) {
                    currentStage = 23;

                }
                break;
            case 23:
                //do nothing
                break;
        }

        // Debug telemetry
        /*telemetryA.addData("Current Stage", currentStage);
        telemetryA.addData("Stage Complete", stageComplete);
        telemetryA.addData("Path Complete", follower.atParametricEnd());
        telemetryA.addData("Motion Complete", isMotionComplete());
        telemetryA.addData("Vert correct", motionSubsystem.vertExtension.atSetPoint());
        telemetryA.addData("Mini Arm correct", motionSubsystem.miniArmPID.atSetPoint());*/

        //follower.telemetryDebug(telemetryA);
        telemetryA.addData("////////CurrentState", currentStage);
        telemetryA.addData("////////Follower Path:", follower.getCurrentPath());
        telemetryA.addData("////////MoveToNext", follower.atParametricEnd() && isMotionComplete());
        telemetryA.addData("////////PathComplete", follower.atParametricEnd());
        telemetryA.addData("////////MotionComplete", isMotionComplete());

        telemetryA.update();
    }
}