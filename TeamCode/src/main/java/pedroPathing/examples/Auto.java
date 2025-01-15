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

@Autonomous(name = "CARTER-AUSTIN", group = "1Examples")
public class Auto extends OpMode {
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

        initializePaths();
        Lineup();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    private void initializePaths() {
        PathBuilder builder;

        // specimen lineup
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(11.000, 64.000, Point.CARTESIAN),
                        new Point(35.5000, 64.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).setPathEndVelocityConstraint(0)
                .build());

        // push prep 0
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
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 30.000, Point.CARTESIAN),
                        new Point(60.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // push 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(60.000, 28.000, Point.CARTESIAN),
                        new Point(26.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        // return 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(26.000, 25.000, Point.CARTESIAN),
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
                        new Point(26.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        // return 2
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(26.000, 20.000, Point.CARTESIAN),
                        new Point(62.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());

        //push prep 3
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(62.000, 20.000, Point.CARTESIAN),
                        new Point(62.000, 12.500, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //push 3
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(62.000, 11.000, Point.CARTESIAN),
                        new Point(25.000, 11.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //speci 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierLine(
                        new Point(26.000, 11.000, Point.CARTESIAN),
                        new Point(12.000, 31.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build());
        //score 1
        builder = new PathBuilder();
        paths.add(builder
                .addPath(new BezierCurve(
                        new Point(12.000, 31.000, Point.CARTESIAN),
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
        boolean miniArmComplete = motionSubsystem.miniArmPID.atSetPoint();

        return  vertComplete && miniArmComplete;
    }

    private void Lineup() { //Lineup
        motionSubsystem.specimenPrep();
        follower.followPath(paths.get(0));
        motionSubsystem.closeClaw();
        //motionSubsystem.setVertTarget(0.5);
        currentStage = 1;
        stageComplete = false;
        follower.setMaxPower(0.75);
    }

    private void PrepZero(){ //Prep 0
        currentStage = 101;
        stageComplete = false;
        motionSubsystem.specimenScore();
        follower.setMaxPower(1);
        follower.holdPoint(new Pose(40.5000, 64.000, Math.toRadians(180)));
    }

    private void PrepTwo() { // Prep 1
        motionSubsystem.clawOpen();
        follower.followPath(paths.get(1));
        motionSubsystem.drivePos();
        currentStage = 2;
        follower.setMaxPower(1);
        stageComplete = false;
    }

    private void PushOne() { // Push 1
        follower.followPath(paths.get(2));
        currentStage = 3;
        motionSubsystem.wallPickupPrep();
        stageComplete = false;
    }

    private void ReturnOne() { // Return 1
        follower.followPath(paths.get(3));
        currentStage = 4;
        stageComplete = false;
    }

    private void PushPrepTwo() { // Push Prep 2
        follower.followPath(paths.get(4));
        currentStage = 5;
        stageComplete = false;
    }

    private void PushTwo() { // Push 2
        follower.followPath(paths.get(5));
        currentStage = 6;
        stageComplete = false;
    }

    private void ReturnTwo() { // Return 2
        follower.followPath(paths.get(6));
        currentStage = 7;
        stageComplete = false;
    }

    private void PushPrepThree() { // Push Prep 3
        follower.followPath(paths.get(7));
        currentStage = 8;
        stageComplete = false;
    }

    private void PushThree() { // Push 3
        follower.followPath(paths.get(8));
        currentStage = 9;
        stageComplete = false;
    }

    private void SpeciOne() { // Speci 1
        follower.followPath(paths.get(9));
        currentStage = 102;
        motionSubsystem.wallPickupPrep();
        stageComplete = false;
    }

    private void SpeciPickupOne(){
        if(!TimerStarted) {
            Timer.reset();
            TimerStarted = true;
        }else if(Timer.seconds() >= 0.5){
            currentStage = 10;
            stageComplete = false;
        }
        motionSubsystem.closeClaw();
    }

    private void ScoreOne() { // Score 1
        motionSubsystem.closeClaw();
        follower.followPath(paths.get(10));
        currentStage = 11;
        stageComplete = false;
    }

    private void SpeciPickupTwo(){
        follower.holdPoint(new Pose(40.500, 62.000, Math.toRadians(180)));
        motionSubsystem.specimenScore();
        currentStage = 103;
        stageComplete = false;
    }

    private void SpeciTwo() { // Speci 2
        motionSubsystem.wallPickupPrep();
        follower.followPath(paths.get(11));
        currentStage = 12;
        stageComplete = false;
    }

    private void SpeciPickupThree(){
        motionSubsystem.closeClaw();
    }

    private void SocreTwo() { // Score 2
        motionSubsystem.closeClaw();
        motionSubsystem.specimenPrep();
        follower.followPath(paths.get(12));
        currentStage = 13;
        stageComplete = false;
    }

    private void SpeciThree() { // Speci 3
        motionSubsystem.wallPickupPrep();
        follower.followPath(paths.get(13));
        currentStage = 14;
        stageComplete = false;
    }

    private void SpeciPickupFour(){}


    private void startStageFifteen() { // Score 3
        motionSubsystem.closeClaw();
        motionSubsystem.specimenPrep();
        follower.followPath(paths.get(14));
        currentStage = 15;
        stageComplete = false;
    } private void startStageSixteen() {
        //follower.followPath(paths.get(15));
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
                    PrepZero();
                }
                break;

            case 101:
                if (isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    PrepTwo();
                }
                break;

            case 2:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    PushOne();
                }
                break;

            case 3:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    ReturnOne();
                }
                break;
            case 4:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    PushPrepTwo();
                }
                break;

            case 5:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    PushTwo();
                }
                break;

            case 6:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    ReturnTwo();
                }
                break;
            case 7:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    PushPrepThree();
                }
                break;

            case 8:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    PushThree();
                }
                break;

            case 9:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    SpeciOne();
                }
                break;

            case 10:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    SpeciPickupOne();
                    motionSubsystem.wallPickupPrep();
                }
                motionSubsystem.wallPickupPrep();
                break;

            case 102:
                if (isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    ScoreOne();
                }
                break; //TODO: we forgot to add this to like 5 of these. Oppsie :(

            case 11:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    SpeciTwo();
                }
                break;

            case 103:
                if (isMotionComplete() && !stageComplete) {
                    stageComplete = true;

                    SocreTwo();
                }
                break;

            case 12:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    SpeciPickupTwo();
                }
                break;
            case 13:
                if (follower.atParametricEnd() && isMotionComplete() && !stageComplete) {
                    stageComplete = true;
                    SpeciThree();
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
        /*telemetryA.addData("Current Stage", currentStage);
        telemetryA.addData("Stage Complete", stageComplete);
        telemetryA.addData("Path Complete", follower.atParametricEnd());
        telemetryA.addData("Motion Complete", isMotionComplete());
        telemetryA.addData("Vert correct", motionSubsystem.vertExtension.atSetPoint());
        telemetryA.addData("Mini Arm correct", motionSubsystem.miniArmPID.atSetPoint());*/

        follower.telemetryDebug(telemetryA);
        telemetryA.update();
    }
}