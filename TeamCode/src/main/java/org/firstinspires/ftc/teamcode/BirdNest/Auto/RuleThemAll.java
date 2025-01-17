package org.firstinspires.ftc.teamcode.BirdNest.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.lib.AutoLib.AutoLib;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.MotionSubsystem;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "RuleThemAll", group = "1Examples")
public class RuleThemAll extends OpMode {

    private enum States {
        TOSUB,
        PUSHPREP1,
        PUSH1,
        PUSHPREP2,
        PUSH2,
        PUSHPREP3,
        PUSH3,
        GRABSPECI1,
        PROCESS_SPECI_ONE,
        SCORE1,
        GRABSPECI2,
        PROCESS_SPECI_TWO,
        SCORE2,
        GRABSPECI3,
        PROCESS_SPECI_THREE,
        SCORE3,
        PARK
    }

    public States autoState = States.TOSUB;

    public static boolean bpath, score_one, score_two, score_three;

    private Follower follower;
    private List<PathChain> paths = new ArrayList<>();
    private Telemetry telemetryA;
    private final Pose startPose = new Pose(11, 64, Math.toRadians(180));

    MotionSubsystem motionSubsystem;
    private int currentStage = 0;
    private boolean stageComplete = false;

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
        score_one = false; score_two = false; score_three = false;

        initializePaths();
        bpath = false;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        follower.update();
        motionSubsystem.loop();

        updateStages();

        // Debug telemetry
        /*telemetryA.addData("Current Stage", currentStage);
        telemetryA.addData("Stage Complete", stageComplete);
        telemetryA.addData("Path Complete", follower.atParametricEnd());
        telemetryA.addData("Motion Complete", isMotionComplete());
        telemetryA.addData("Vert correct", motionSubsystem.vertExtension.atSetPoint());
        telemetryA.addData("Mini Arm correct", motionSubsystem.miniArmPID.atSetPoint());*/

        telemetryA.addData("AUTOSTATE: ", autoState);

        follower.telemetryDebug(telemetryA);
        telemetryA.update();
    }

    private boolean isMotionComplete() {
        boolean vertComplete = motionSubsystem.vertExtension.atSetPoint();
        boolean miniArmComplete = motionSubsystem.miniArmPID.atSetPoint();

        return  vertComplete && miniArmComplete;
    }

    private boolean isStageComplete() {
        return isMotionComplete() && follower.atParametricEnd();
    }

    private void updateStages(){
        switch (autoState) {
            case TOSUB:
                if(!bpath){
                    Timer.reset();
                    bpath = true;
                }
                if(Timer.seconds() < 0.05){follower.followPath(paths.get(0));}

                motionSubsystem.specimenPrep();
                motionSubsystem.closeClaw();
                follower.setMaxPower(1);

                if(!follower.isBusy()) {
                    motionSubsystem.specimenPrep();
                    autoState = States.PUSHPREP1;
                    Timer.reset();
                }
                break;

            case PUSHPREP1:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(1));
                }

                if(!follower.isBusy()) {
                    autoState = States.PUSH1;
                    Timer.reset();
                }
                break;

            case PUSH1:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(2));
                }

                motionSubsystem.wallPickupPrep();

                if(!follower.isBusy()) {
                    autoState = States.PUSHPREP2;
                    Timer.reset();
                }
                break;

            case PUSHPREP2:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(3));
                }

                if(!follower.isBusy()) {
                    Timer.reset();
                    autoState = States.PUSH2;
                }
                break;

            case PUSH2:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(4));
                }

                if(!follower.isBusy()) {
                    autoState = States.PUSHPREP3;
                    Timer.reset();
                }
                break;

            case PUSHPREP3:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(5));
                }

                if(!follower.isBusy()) {
                    autoState = States.PUSH3;
                    Timer.reset();
                }
                break;

            case PUSH3:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(6));
                }

                if(!follower.isBusy()) {
                    autoState = States.GRABSPECI1;
                    Timer.reset();
                }
                break;

            case GRABSPECI1:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(7));
                }

                if(!follower.isBusy()) {
                    autoState = States.PROCESS_SPECI_ONE;
                    Timer.reset();
                }
                break;

            case PROCESS_SPECI_ONE:
                if(Timer.seconds() < 0.05){
                    //follower.followPath(paths.get(1));
                }

                if(Timer.seconds() > 0.2 && Timer.seconds() < 0.25){
                    motionSubsystem.closeClaw();
                }

                if (Timer.seconds() > 0.3){
                    Timer.reset();
                    autoState = States.SCORE1;
                }
                break;

            case SCORE1:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(8));
                }

                if (!follower.isBusy() && Timer.seconds() > 0.2){
                    motionSubsystem.specimenPrep();

                    if (!score_one) {Timer.reset(); score_one = true;}

                    if(score_one && Timer.seconds() > 0.6){
                        motionSubsystem.clawOpen();
                        autoState = States.GRABSPECI2;
                    }
                }
                break;

            case GRABSPECI2:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(9));
                }

                motionSubsystem.wallPickupPrep();

                if(!follower.isBusy()) {
                    autoState = States.PROCESS_SPECI_ONE;
                    Timer.reset();
                }
                break;

            case PROCESS_SPECI_TWO:
                //follower.holdPoint(follower.getPose());

                if(Timer.seconds() > 0.2 && Timer.seconds() < 0.25){
                    motionSubsystem.closeClaw();
                }

                if (Timer.seconds() > 0.3){
                    Timer.reset();
                    autoState = States.SCORE1;
                }
                break;

            case SCORE2:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(10));
                }

                if (!follower.isBusy() && Timer.seconds() > 0.2){
                    motionSubsystem.specimenPrep();

                    if (!score_two) {Timer.reset(); score_two = true;}

                    if(score_two && Timer.seconds() > 0.6){
                        motionSubsystem.clawOpen();
                        autoState = States.GRABSPECI2;
                    }
                }
                break;

            case GRABSPECI3:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(11));
                }

                motionSubsystem.wallPickupPrep();

                if(!follower.isBusy()) {
                    autoState = States.PROCESS_SPECI_ONE;
                    Timer.reset();
                }
                break;

            case PROCESS_SPECI_THREE:
                //follower.holdPoint(follower.getPose());

                if(Timer.seconds() > 0.2 && Timer.seconds() < 0.25){
                    motionSubsystem.closeClaw();
                }

                if (Timer.seconds() > 0.3){
                    Timer.reset();
                    autoState = States.SCORE1;
                }
                break;

            case SCORE3:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(12));
                }

                if (!follower.isBusy() && Timer.seconds() > 0.2){
                    motionSubsystem.specimenPrep();

                    if (!score_three) {Timer.reset(); score_three = true;}

                    if(score_one && Timer.seconds() > 0.6){
                        motionSubsystem.clawOpen();
                        autoState = States.GRABSPECI2;
                    }
                }
                break;

            case PARK:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(13));
                }

        }
    }

    private void initializePaths() {

        PathBuilder toSub = new PathBuilder();
        paths.add(toSub
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(11.000, 64.000, Point.CARTESIAN),
                                new Point(35.000, 64.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushPrepOne = new PathBuilder();
        paths.add(pushPrepOne
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(35.000, 64.000, Point.CARTESIAN),
                                new Point(16.000, 48.000, Point.CARTESIAN),
                                new Point(60.000, 28.000, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushOne = new PathBuilder();
        paths.add(pushOne
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(60.000, 28.000, Point.CARTESIAN),
                                new Point(26.000, 28.000, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushPrepTwo = new PathBuilder();
        paths.add(pushPrepTwo
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(26.000, 28.000, Point.CARTESIAN),
                                new Point(60.000, 29.400, Point.CARTESIAN),
                                new Point(60.000, 20.000, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushTwo = new PathBuilder();
        paths.add(pushTwo
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(60.000, 20.000, Point.CARTESIAN),
                                new Point(26.000, 20.000, Point.CARTESIAN)
                        )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushPrepThree = new PathBuilder();
        paths.add(pushPrepThree
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(26.000, 20.000, Point.CARTESIAN),
                                new Point(60.000, 20.800, Point.CARTESIAN),
                                new Point(60.000, 11.000, Point.CARTESIAN)
                        )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushThree = new PathBuilder();
        paths.add(pushThree
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(60.000, 11.000, Point.CARTESIAN),
                                new Point(25.000, 11.000, Point.CARTESIAN)
                        )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder grabSepciOne = new PathBuilder();
        paths.add(grabSepciOne
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(25.000, 11.000, Point.CARTESIAN),
                                new Point(24.000, 24.000, Point.CARTESIAN),
                                new Point(12.250, 30.250, Point.CARTESIAN)
                        )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder scoreOne = new PathBuilder();
        paths.add(scoreOne
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(12.250, 30.250, Point.CARTESIAN),
                                new Point(14.000, 62.000, Point.CARTESIAN),
                                new Point(32.000, 62.000, Point.CARTESIAN)
                        )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder grabSepciTwo = new PathBuilder();
        paths.add(grabSepciTwo
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(32.000, 62.000, Point.CARTESIAN),
                                new Point(12.250, 34.500, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder scoreTwo = new PathBuilder();
        paths.add(scoreTwo
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(12.250, 34.500, Point.CARTESIAN),
                                new Point(14.000, 66.000, Point.CARTESIAN),
                                new Point(32.000, 66.500, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder grabSepciThree = new PathBuilder();
        paths.add(grabSepciThree
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(32.000, 66.500, Point.CARTESIAN),
                                new Point(12.250, 40.000, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder scoreThree = new PathBuilder();
        paths.add(scoreThree
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(12.250, 40.000, Point.CARTESIAN),
                                new Point(11.750, 64.750, Point.CARTESIAN),
                                new Point(31.750, 71.500, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder park = new PathBuilder();
        paths.add(park
                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(31.750, 71.500, Point.CARTESIAN),
                                new Point(8.000, 19.250, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
    }
}