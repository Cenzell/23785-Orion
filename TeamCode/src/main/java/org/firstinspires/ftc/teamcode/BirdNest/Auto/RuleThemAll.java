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
        PARK,
        COM;
    }

    public States autoState;

    public static boolean bpath, score_one, score_two, score_three, clipOne, clipTwo;

    private Follower follower;
    private List<PathChain> paths = new ArrayList<>();
    private Telemetry telemetryA;
    private final Pose startPose = new Pose(11, 64, Math.toRadians(180));

    MotionSubsystem motionSubsystem;

    private ElapsedTime Timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motionSubsystem = new MotionSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        motionSubsystem.init();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        score_one = false; score_two = false; score_three = false; clipOne = false; clipTwo = false;

        autoState = States.TOSUB;

        initializePaths();
        bpath = false;

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        Timer.reset();
        motionSubsystem.closeClaw();
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

        telemetryA.addData("/////AUTOSTATE: /////", autoState);

        follower.telemetryDebug(telemetryA);
        telemetryA.update();
    }

    private boolean isMotionComplete() {
        boolean vertComplete = motionSubsystem.vertExtension.atSetPoint();
        boolean miniArmComplete = motionSubsystem.miniArmPID.atSetPoint();

        return  vertComplete && miniArmComplete;
    }

    private void updateStages(){
        switch (autoState) {
            case TOSUB:
                if(!bpath){
                    Timer.reset();
                    bpath = true;
                }
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(0));
                    follower.setMaxPower(0.9);
                }

                motionSubsystem.specimenPrep();
                follower.setMaxPower(1);

                if(!follower.isBusy()) {
                    autoState = States.PUSHPREP1;
                    Timer.reset();
                }
                break;

            case PUSHPREP1:
                if (Timer.seconds() < 0.05){
                    follower.setMaxPower(1);
                }

                if (Timer.seconds() < 1.4){
                    motionSubsystem.specimenScore();
                }

                if (Timer.seconds() > 1.4 && Timer.seconds() < 1.56){
                    motionSubsystem.clawOpen();
                }

                if (Timer.seconds() > 1.75 && Timer.seconds() < 2){
                    motionSubsystem.drivePos();
                }

                if(Timer.seconds() > 2 && Timer.seconds() < 2.1){
                    motionSubsystem.wallPickupPrep();
                    follower.followPath(paths.get(1));
                }

                if(Timer.seconds() > 3.1 && !follower.isBusy()) {
                    autoState = States.PUSH1;
                    Timer.reset();
                }
                break;

            case PUSH1:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(2));
                }

                motionSubsystem.drivePos();

                if(!follower.isBusy()) {
                    autoState = States.PUSHPREP2;
                    Timer.reset();
                }
                break;

            case PUSHPREP2:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(3));
                }

                motionSubsystem.wallPickupPrep();

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
                    autoState = States.GRABSPECI1;
                    Timer.reset();
                }
                break;

            /*case PUSHPREP3:
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
                break;*/

            case GRABSPECI1:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(5));
                }

                if(!follower.isBusy()) {
                    motionSubsystem.Claw.setPosition(0.3);
                    Timer.reset();
                    autoState = States.PROCESS_SPECI_ONE;
                }
                break;

            case PROCESS_SPECI_ONE:
                if(Timer.seconds() < 0.05){
                    motionSubsystem.setVertTarget(0.1);
                }

                motionSubsystem.closeClaw();

                if(Timer.seconds() > 0.2 && Timer.seconds() < 0.6){
                    motionSubsystem.closeClaw();
                }

                if(Timer.seconds() > 0.6 && Timer.seconds() < 1){
                    motionSubsystem.setVertTarget(3);
                }

                if (Timer.seconds() > 1){
                    Timer.reset();
                    autoState = States.SCORE1;
                }
                break;

            case SCORE1:
                if(!score_one && Timer.seconds() < 0.05){
                    follower.followPath(paths.get(6));
                    motionSubsystem.drivePos();
                }
                if(!score_one && Timer.seconds() < 0.1 && Timer.seconds() > 0.05){
                    motionSubsystem.Claw.setPosition(0.3);
                }


                if (!follower.isBusy() && Timer.seconds() > 0.2){

                    if (!score_one) {Timer.reset(); score_one = true; motionSubsystem.specimenPrep();}

                    if(score_one && Timer.seconds() < 1){
                        motionSubsystem.specimenScore();
                    }

                    if(score_one && Timer.seconds() > 1){
                        motionSubsystem.clawOpen();
                        autoState = States.GRABSPECI2;
                    }
                }
                break;

            case GRABSPECI2:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(7));
                }

                motionSubsystem.wallPickupPrep();

                if(!follower.isBusy()) {
                    autoState = States.PROCESS_SPECI_TWO;
                    Timer.reset();
                }
                break;

            case PROCESS_SPECI_TWO:
                //follower.holdPoint(follower.getPose());

                if(Timer.seconds() > 0.2 && Timer.seconds() < 0.5){
                    motionSubsystem.closeClaw();
                }

                if (Timer.seconds() > 0.3){
                    Timer.reset();
                    autoState = States.SCORE2;
                }
                break;

            case SCORE2:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(8));
                }

                if (!follower.isBusy() && Timer.seconds() > 0.2){
                    motionSubsystem.specimenPrep();

                    if (!score_two) {Timer.reset(); score_two = true;}

                    if(score_two && Timer.seconds() > 0.6){
                        motionSubsystem.clawOpen();
                        autoState = States.PARK;
                    }
                }
                break;

            /*case GRABSPECI3:
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
                break;*/

            case PARK:
                if(Timer.seconds() < 0.05){
                    follower.followPath(paths.get(9));
                }
            case COM:
                break;

        }
    }

    private void initializePaths() {

        PathBuilder toSub = new PathBuilder();
        paths.add(toSub
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(11.000, 64.500, Point.CARTESIAN),
                                new Point(35, 64.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushPrepOne = new PathBuilder();
        paths.add(pushPrepOne
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(35, 64.250, Point.CARTESIAN),
                                new Point(21.3, 39.7, Point.CARTESIAN),
                                new Point(32.9, 35, Point.CARTESIAN),
                                new Point(66.400, 33, Point.CARTESIAN),
                                new Point(66.5,31.2, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushOne = new PathBuilder();
        paths.add(pushOne
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(65.000, 32, Point.CARTESIAN),
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
                                new Point(64.000, 26.000, Point.CARTESIAN),
                                new Point(67.000, 30.600, Point.CARTESIAN),
                                new Point(66.600, 19.9, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushTwo = new PathBuilder();
        paths.add(pushTwo
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(66.500, 19.900, Point.CARTESIAN),
                                new Point(25.000, 14, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        /*PathBuilder pushPrepThree = new PathBuilder();
        paths.add(pushPrepThree
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(26.000, 20.000, Point.CARTESIAN),
                                new Point(67.000, 19.500, Point.CARTESIAN),
                                new Point(67.500, 22.000, Point.CARTESIAN),
                                new Point(66.800, 14, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder pushThree = new PathBuilder();
        paths.add(pushThree
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(66.800, 14, Point.CARTESIAN),
                                new Point(25.000, 14, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());*/

        PathBuilder grabSepciOne = new PathBuilder();
        paths.add(grabSepciOne
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(25.000, 14, Point.CARTESIAN),
                                new Point(26, 29.23, Point.CARTESIAN),
                                new Point(13.85, 29.9, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder scoreOne = new PathBuilder();
        paths.add(scoreOne
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(13.85, 29.9, Point.CARTESIAN),
                                new Point(14.000, 62.000, Point.CARTESIAN),
                                new Point(34.200, 62.000, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder grabSepciTwo = new PathBuilder();
        paths.add(grabSepciTwo
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(42.200, 62.000, Point.CARTESIAN),
                                new Point(13.85, 29.9, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder scoreTwo = new PathBuilder();
        paths.add(scoreTwo
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(13.85, 29.9, Point.CARTESIAN),
                                new Point(14.000, 66.000, Point.CARTESIAN),
                                new Point(31.750, 71.500, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        /*PathBuilder grabSepciThree = new PathBuilder();
        paths.add(grabSepciThree
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(32.000, 66.500, Point.CARTESIAN),
                                new Point(13, 40.000, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        PathBuilder scoreThree = new PathBuilder();
        paths.add(scoreThree
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(13, 40.000, Point.CARTESIAN),
                                new Point(11.750, 64.750, Point.CARTESIAN),
                                new Point(31.750, 71.500, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());*/

        PathBuilder park = new PathBuilder();
        paths.add(park
                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(31.750, 68, Point.CARTESIAN),
                                new Point(8.000, 19.250, Point.CARTESIAN)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
    }
}