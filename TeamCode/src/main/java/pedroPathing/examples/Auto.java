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

import java.util.ArrayList;
import java.util.List;



@Autonomous(name = "CARTER-AUSTIN", group = "Examples")
public class Auto extends OpMode {
    private Follower follower;
    private List<PathChain> paths = new ArrayList<>();
    private Telemetry telemetryA;
    private final Pose startPose = new Pose(11, 64, Math.toRadians(180));

    MotionSubsystem motionSubsystem;
    private int currentStage = 0;
    private boolean stageComplete = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motionSubsystem = new MotionSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        motionSubsystem.init();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        initializePaths();
        startStageOne();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void initializePaths() {
        PathBuilder builder;

        // specimen lineup
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(11.000, 64.000, Point.CARTESIAN),
                        new Point(32.000, 64.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push prep 0
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(32.000, 64.000, Point.CARTESIAN),
                        new Point(16.000, 42.000, Point.CARTESIAN),
                        new Point(60.000, 30.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push prep 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 30.000, Point.CARTESIAN),
                        new Point(60.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 25.000, Point.CARTESIAN),
                        new Point(25.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // return 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(25.000, 25.000, Point.CARTESIAN),
                        new Point(60.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push prep 2
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 29.000, Point.CARTESIAN),
                        new Point(60.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push 2
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 20.000, Point.CARTESIAN),
                        new Point(25.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        // return 2
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(25.000, 20.000, Point.CARTESIAN),
                        new Point(60.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //push prep 3
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 20.000, Point.CARTESIAN),
                        new Point(60.000, 15.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //push 3
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 15.000, Point.CARTESIAN),
                        new Point(25.000, 15.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //speci 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(25.000, 16.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //score 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 62.000, Point.CARTESIAN),
                        new Point(32.000, 62.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // speci 2
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(32.000, 62.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //score 2
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 66.000, Point.CARTESIAN),
                        new Point(32.000, 66.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // speci 3
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 70.000, Point.CARTESIAN),
                        new Point(32.000, 70.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //score 3
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(32.000, 70.000, Point.CARTESIAN),
                        new Point(8.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
    }

    private boolean isMotionComplete() {
        boolean vertComplete = motionSubsystem.vertExtension.atSetPoint();
        boolean wristComplete = motionSubsystem.wristPID.atSetPoint();
        boolean miniArmComplete = motionSubsystem.miniArmPID.atSetPoint();

        return  vertComplete && /*wristComplete &&*/ miniArmComplete;
    }

    private void startStageOne() {
        motionSubsystem.specimenPrep();
        follower.followPath(paths.get(0));
        currentStage = 1;
        stageComplete = false;
    }

    private void startStageTwo() {
        follower.followPath(paths.get(1));
        currentStage = 2;
        stageComplete = false;
    }

    private void startStageThree() {
        follower.followPath(paths.get(2));
        currentStage = 3;
        stageComplete = false;
    }

    private void startStageFour() {
        follower.followPath(paths.get(3));
        currentStage = 4;
        stageComplete = false;
    }

    private void startStageFive() {
        follower.followPath(paths.get(4));
        currentStage = 5;
        stageComplete = false;
    }

    private void startStageSix() {
        follower.followPath(paths.get(5));
        currentStage = 6;
        stageComplete = false;
    }

    private void startStageSeven() {
        follower.followPath(paths.get(6));
        currentStage = 7;
        stageComplete = false;
    }

    private void startStageEight() {
        follower.followPath(paths.get(7));
        currentStage = 8;
        stageComplete = false;
    }

    private void startStageNine() {
        follower.followPath(paths.get(8));
        currentStage = 9;
        stageComplete = false;
    }

    private void startStageTen() {
        follower.followPath(paths.get(9));
        currentStage = 10;
        stageComplete = false;
    }

    private void startStageEleven() {
        follower.followPath(paths.get(10));
        currentStage = 11;
        stageComplete = false;
    }

    private void startStageTwelve() {
        follower.followPath(paths.get(11));
        currentStage = 12;
        stageComplete = false;
    }
    private void startStageThirteen() {
        follower.followPath(paths.get(12));
        currentStage = 13;
        stageComplete = false;
    }

    private void startStageFourteen() {
        follower.followPath(paths.get(13));
        currentStage = 14;
        stageComplete = false;
    }
    private void startStageFifteen() {
        follower.followPath(paths.get(14));
        currentStage = 15;
        stageComplete = false;
    } private void startStageSixteen() {
        follower.followPath(paths.get(15));
        currentStage = 16;
        stageComplete = false;
    }


    @Override
    public void loop() {
        follower.update();
        motionSubsystem.loop();

        switch (currentStage) {
            case 1:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageTwo();
                }
                break;

            case 2:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageThree();
                }
                break;

            case 3:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageFour();
                }
                break;
            case 4:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageFive();
                }
                break;

            case 5:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageSix();
                }
                break;

            case 6:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageSeven();
                }
                break;
            case 7:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageEight();
                }
                break;

            case 8:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageNine();
                }
                break;

            case 9:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageTen();
                }
                break;

            case 10:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageEleven();
                }
                break;

            case 11:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageTwelve();
                }
                break;

            case 12:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageThirteen();
                }
                break;
            case 13:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageFourteen();
                }
                break;
            case 14:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageFifteen();
                }
                break;
            case 15:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    startStageSixteen();
                }
                break;
            case 16:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                }
                break;
        }

        // Debug telemetry
        telemetryA.addData("Current Stage", currentStage);
        telemetryA.addData("Stage Complete", stageComplete);
        telemetryA.addData("Path Complete", follower.atParametricEnd());
        telemetryA.addData("Motion Complete", isMotionComplete());
        telemetryA.addData("Vert correct", motionSubsystem.vertExtension.atSetPoint());
        telemetryA.addData("Wrist correct", motionSubsystem.wristPID.atSetPoint());
        telemetryA.addData("Mini Arm correct", motionSubsystem.miniArmPID.atSetPoint());

        follower.telemetryDebug(telemetryA);
        telemetryA.update();
    }
}