package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.MotionSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.List;



@Autonomous(name = "BG", group = "Examples")
public class BGAuto extends OpMode {

    private enum AutoState {
        LINEUP,
        PREP_ZERO,
        PREP_ONE,
        PUSH_ONE,
        RETURN_ONE,
        PREP_TWO,
        PUSH_TWO,
        RETURN_TWO,
        PREP_THREE,
        PUSH_THREE,
        SPECI_ONE,
        SCORE_ONE,
        SPECI_TWO,
        SCORE_TWO,
        SPECI_THREE,
        SCORE_THREE,
        SPECI_FOUR,
        SCORE_FOUR,
        SPECI_FIVE,
        SCORE_FIVE;
    }

    private AutoState currentState = AutoState.LINEUP;
    private ElapsedTime stateTimer = new ElapsedTime();

    private Follower follower;
    private Path[] paths = new Path[16];
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

        follower.setPose(new Pose(11,64, Math.toRadians(180)));

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        stateTimer.reset();
    }

    public PathBuilder lineup, prep0, prep1, prep2, prep3, prep4, prep5, push1, push2, push3, push4, push5, return1, return2, return3, speci_one, speci_two, speci_three, speci_four, speci_five, score_one, score_two, score_three;

    private void initializePaths() {

        // specimen lineup
        lineup = new PathBuilder();
        lineup.addPath(new BezierLine(
                        new Point(11.000, 64.000, Point.CARTESIAN),
                        new Point(32.000, 64.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain0 = lineup.build();
        paths[0] = chain0.getPath(0);

        // push prep 0
        prep0 = new PathBuilder();
        prep0
                .addPath(new BezierCurve(
                        new Point(32.000, 64.000, Point.CARTESIAN),
                        new Point(16.000, 42.000, Point.CARTESIAN),
                        new Point(60.000, 30.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain1 = prep0.build();
        paths[1] = chain1.getPath(0);

        // push prep 1
        prep1 = new PathBuilder();
        prep1
                .addPath(new BezierLine(
                        new Point(60.000, 30.000, Point.CARTESIAN),
                        new Point(60.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain2 = prep1.build();
        paths[2] = chain2.getPath(0);

        // push 1
        push1 = new PathBuilder();
        push1
                .addPath(new BezierLine(
                        new Point(60.000, 25.000, Point.CARTESIAN),
                        new Point(25.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain3 = push1.build();
        paths[3] = chain3.getPath(0);

        // return 1
        return1 = new PathBuilder();
        return1
                .addPath(new BezierLine(
                        new Point(25.000, 25.000, Point.CARTESIAN),
                        new Point(60.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain4 = return1.build();
        paths[4] = chain4.getPath(0);

        // push prep 2
        prep2 = new PathBuilder();
        prep2
                .addPath(new BezierLine(
                        new Point(60.000, 29.000, Point.CARTESIAN),
                        new Point(60.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain5 = return1.build();
        paths[5] = chain5.getPath(0);

        // push 2
        push2 = new PathBuilder();
        push2
                .addPath(new BezierLine(
                        new Point(60.000, 20.000, Point.CARTESIAN),
                        new Point(25.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chaina = return1.build();
        paths[6] = chaina.getPath(0);

        // return 2
        return2 = new PathBuilder();
        return2
                .addPath(new BezierLine(
                        new Point(25.000, 20.000, Point.CARTESIAN),
                        new Point(60.000, 20.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain6 = return1.build();
        paths[7] = chain6.getPath(0);

        //push prep 3
        prep3 = new PathBuilder();
        prep3
                .addPath(new BezierLine(
                        new Point(60.000, 20.000, Point.CARTESIAN),
                        new Point(60.000, 15.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain7 = return1.build();
        paths[8] = chain7.getPath(0);

        //push 3
        push3 = new PathBuilder();
        push3
                .addPath(new BezierLine(
                        new Point(60.000, 15.000, Point.CARTESIAN),
                        new Point(25.000, 15.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain8 = return1.build();
        paths[9] = chain8.getPath(0);

        //speci 1
        speci_one = new PathBuilder();
        speci_one
                .addPath(new BezierLine(
                        new Point(25.000, 16.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain9 = return1.build();
        paths[10] = chain9.getPath(0);

        //score 1
        score_one = new PathBuilder();
        score_one
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 62.000, Point.CARTESIAN),
                        new Point(32.000, 62.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain10 = return1.build();
        paths[11] = chain10.getPath(0);

        // speci 2
        speci_two = new PathBuilder();
        speci_two
                .addPath(new BezierLine(
                        new Point(32.000, 62.000, Point.CARTESIAN),
                        new Point(12.000, 35.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain11 = return1.build();
        paths[12] = chain11.getPath(0);

        //score 2
        score_two = new PathBuilder();
        score_two
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 66.000, Point.CARTESIAN),
                        new Point(32.000, 66.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain12 = return1.build();
        paths[13] = chain12.getPath(0);

        // speci 3
        speci_three = new PathBuilder();
        speci_three
                .addPath(new BezierCurve(
                        new Point(12.000, 35.000, Point.CARTESIAN),
                        new Point(14.000, 70.000, Point.CARTESIAN),
                        new Point(32.000, 70.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain13 = return1.build();
        paths[14] = chain13.getPath(0);

        //score 3
        score_three = new PathBuilder();
        score_three
                .addPath(new BezierLine(
                        new Point(32.000, 70.000, Point.CARTESIAN),
                        new Point(8.000, 28.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        PathChain chain14 = return1.build();
        paths[15] = chain14.getPath(0);
    }

    private boolean isMotionComplete() {
        boolean vertComplete = motionSubsystem.vertExtension.atSetPoint();
        boolean miniArmComplete = motionSubsystem.miniArmPID.atSetPoint();

        return  vertComplete && miniArmComplete;
    }

    public void updateState(){
        switch (currentState){
            case LINEUP:
                if(!follower.isBusy()){
                    follower.followPath(paths[0]);}
                    motionSubsystem.specimenPrep();
                if ((follower.atParametricEnd() && isMotionComplete())) {
                    currentState = AutoState.PREP_ZERO;
                }
                break;
            case PREP_ZERO:
                if(!follower.isBusy()){
                    follower.followPath(paths[1]);}
                if (follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.PREP_ONE;
                }
                break;
            case PREP_ONE:
                follower.followPath(paths[2]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.PUSH_ONE;
                }
                break;
            case PUSH_ONE:
                follower.followPath(paths[3]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.RETURN_ONE;
                }
                break;
            case RETURN_ONE:
                follower.followPath(paths[4]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.PREP_TWO;
                }
                break;
            case PREP_TWO:
                follower.followPath(paths[5]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.PUSH_TWO;
                }
                break;
            case PUSH_TWO:
                follower.followPath(paths[6]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.RETURN_TWO;
                }
                break;
            case PREP_THREE:
                follower.followPath(paths[7]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.PUSH_THREE;
                }
                break;
            case PUSH_THREE:
                follower.followPath(paths[8]);
                if (follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.SPECI_ONE;
                }
                break;
            case SPECI_ONE:
                follower.followPath(paths[9]);
                if (follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.SCORE_ONE;
                }
                break;
            case SCORE_ONE:
                follower.followPath(paths[10]);
                if (follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.SPECI_TWO;
                }
                break;
            case SPECI_TWO:
                follower.followPath(paths[11]);
                if (follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.SCORE_TWO;
                }
                break;
            case SCORE_TWO:
                follower.followPath(paths[12]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.SPECI_THREE;
                }
                break;
            case SPECI_THREE:
                follower.followPath(paths[13]);
                if(follower.atParametricEnd() && isMotionComplete()){
                    currentState = AutoState.SCORE_THREE;
                }
                break;
            case SCORE_THREE:
                follower.followPath(paths[14]);
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        motionSubsystem.loop();

        updateState();

        // Debug telemetry
        telemetryA.addData("Current Stage", currentState);
        telemetryA.addData("Path Complete", follower.atParametricEnd());
        telemetryA.addData("Motion Complete", isMotionComplete());
        telemetryA.addData("Vert correct", motionSubsystem.vertExtension.atSetPoint());
        telemetryA.addData("Mini Arm correct", motionSubsystem.miniArmPID.atSetPoint());

        follower.telemetryDebug(telemetryA);
        telemetryA.update();
    }
}