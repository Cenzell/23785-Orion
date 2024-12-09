package org.firstinspires.ftc.teamcode.BigArm.AUTO;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BigArm.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.BigArm.lib.pedroPathing.pathGeneration.Point;

@Config
@Autonomous(name = "Auto Cool Rev.2", group = "Orion")
public class goodest_v2 extends OpMode {
    private enum AutoState {
        INIT,
        APPROACH_SUBSTATION,
        SCORE_FIRST,
        DRIVE_TO_PICKUP_1,
        GRAB_PIECE_1,
        TURN_TO_SCORE_1,
        EXTEND_ARM_1,
        SCORE_PIECE_1,
        RETRACT_AND_TURN_1,
        DRIVE_TO_PICKUP_2,
        GRAB_PIECE_2,
        TURN_TO_SCORE_2,
        EXTEND_ARM_2,
        SCORE_PIECE_2,
        RETRACT_AND_TURN_2,
        DRIVE_TO_PICKUP_3,
        GRAB_PIECE_3,
        TURN_TO_SCORE_3,
        EXTEND_ARM_3,
        SCORE_PIECE_3,
        RETRACT_FOR_PARK,
        PAUSE,
        PARK,
        DONE
    }

    private AutoState currentState = AutoState.INIT;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private DcMotor armMotorLeft, armMotorRight;
    private DcMotor extenderLeft, extenderRight;
    private CRServo intakeServo;

    private final InterpLUT IntP = new InterpLUT(), IntI = new InterpLUT(),
            IntD = new InterpLUT(), IntF = new InterpLUT();
    private PIDFController armPid;
    private static PIDController extenderPid;

    private static final double TICKS_PER_DEGREE = 360.0/8192.0;
    private static final double EXTENDER_TICKS = (10.0 / 257.5)/3;

    public static double[] pid1 = {0.035, 0, 0, 0.0009};
    public static double[] pid2 = {0.035, 0, 0, 0.0009};
    public static double[] pide = {0.08, 0, 0};
    public static double power_mult = 0.85;

    private double armTarget = 0;
    private double extenderTarget = 16;
    private Path[] paths = new Path[7];

    IntakeSubsystem intakeSubsystem;

    @Override
    public void init() {
        intakeSubsystem = new IntakeSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        intakeSubsystem.init();
        try {
            initHardware();
            initPID();
            buildAllPaths();
            follower.setPose(new Pose(10.000, 60, 0));
            runtime.reset();
            stateTimer.reset();
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
            requestOpModeStop();//AHHHHHHHHHH
        }
    }

    private void buildAllPaths() {
        // Path to substation
        PathBuilder toSub = new PathBuilder();
        toSub.addPath(new BezierLine(
                new Point(10.000, 60, Point.CARTESIAN),
                new Point(35.5, 60, Point.CARTESIAN)
        )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        PathChain chain0 = toSub.build();
        paths[0] = chain0.getPath(0);  // Get first path from chain

        // Path to piece
        PathBuilder toPiece = new PathBuilder();
        toPiece.addPath(new BezierLine(
                new Point(35.5, 60.000, Point.CARTESIAN),
                new Point(24.500, 100, Point.CARTESIAN)
        )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(2));
        PathChain chain1 = toPiece.build();
        paths[1] = chain1.getPath(0);

        // Path to scoring position
        PathBuilder toScore = new PathBuilder();
        toScore.addPath(new BezierLine(
                new Point(24.000, 100, Point.CARTESIAN),
                new Point(22.000, 100, Point.CARTESIAN)
        )).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        PathChain chain2 = toScore.build();
        paths[2] = chain2.getPath(0);

        // Path to second piece
        PathBuilder toPieceTwo = new PathBuilder();
        toPieceTwo.addPath(new BezierLine(
                new Point(22.000, 100, Point.CARTESIAN),
                new Point(24.000, 98.500, Point.CARTESIAN)
        )).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(32));
        PathChain chain3 = toPieceTwo.build();
        paths[3] = chain3.getPath(0);

        // Path to third piece
        PathBuilder toPieceThree = new PathBuilder();
        toPieceThree.addPath(new BezierLine(
                new Point(24.000, 98.500, Point.CARTESIAN),
                new Point(24.000, 104, Point.CARTESIAN)
        )).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(43));
        PathChain chain4 = toPieceThree.build();
        paths[4] = chain4.getPath(0);

        // Path to park
        PathBuilder park = new PathBuilder();
        park.addPath(new BezierCurve(
                new Point(20.000, 100, Point.CARTESIAN),
                new Point(80.000, 110.000, Point.CARTESIAN),
                new Point(80.000, 80.000, Point.CARTESIAN)
        )).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90));
        PathChain chain5 = park.build();
        paths[5] = chain5.getPath(0);
    }

    @Override
    public void loop() {
        intakeSubsystem.loop();

        telemetry.addData("State:", AutoState.class.getName());

        updateSensors();
        updateState();
        updateActuators();
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("State Timer", "%.1f seconds", stateTimer.seconds());
        telemetry.addData("Arm Target", armTarget);
        telemetry.addData("Extender Target", extenderTarget);
        telemetry.update();
    }

    private void updateState() {
        switch (currentState) {
            case INIT:
                if (stateTimer.seconds() > 1.2) {
                    currentState = AutoState.APPROACH_SUBSTATION;
                    stateTimer.reset();
                }
                armTarget = 64;
                extenderTarget = 64;
                break;

            case APPROACH_SUBSTATION:
                if (!follower.isBusy()) {
                    follower.followPath(paths[0]);
                }
                if (stateTimer.seconds() > 1) {
                    currentState = AutoState.SCORE_FIRST;
                    stateTimer.reset();
                }
                break;

            case SCORE_FIRST:
                extenderTarget = 16;
                armTarget = 66;
                if (stateTimer.seconds() > 1.0) {
                    currentState = AutoState.DRIVE_TO_PICKUP_1;
                    stateTimer.reset();
                }
                break;

            case DRIVE_TO_PICKUP_1:
                armTarget = 13.5;
                intakeServo.setPower(-1);
                if (!follower.isBusy()) {
                    follower.followPath(paths[1]);
                }
                if (stateTimer.seconds() > 1.75) {
                    currentState = AutoState.PAUSE;
                    stateTimer.reset();
                }
                break;

            case PAUSE:
                intakeServo.setPower(1);
                if (!follower.isBusy()) {
                    follower.followPath(paths[1]);
                }
                if (stateTimer.seconds() > 1) {
                    currentState = AutoState.GRAB_PIECE_1;
                    stateTimer.reset();
                }
                break;

            case GRAB_PIECE_1:
                intakeServo.setPower(1);
                armTarget = 13.5;
                extenderTarget = 33;
                if(intakeSubsystem.hasPiece() == IntakeSubsystem.Possession.HAS_PIECE){
                    currentState = AutoState.TURN_TO_SCORE_1;
                    stateTimer.reset();
                }
                if (stateTimer.seconds() > 2.25) {
                    currentState = AutoState.TURN_TO_SCORE_1;
                    stateTimer.reset();
                }
                break;

            case TURN_TO_SCORE_1:
                intakeServo.setPower(0);
                extenderTarget = 16;
                if (!follower.isBusy()) {
                    follower.followPath(paths[2]);
                }
                armTarget = 100;
                if (stateTimer.seconds() > 1) {
                    currentState = AutoState.EXTEND_ARM_1;
                    stateTimer.reset();
                }
                break;

            case EXTEND_ARM_1:
                armTarget=88;
                extenderTarget = 64;
                if (stateTimer.seconds() > 0.8) {
                    currentState = AutoState.SCORE_PIECE_1;
                    stateTimer.reset();
                }
                break;

            case SCORE_PIECE_1:
                intakeServo.setPower(-1);
                if(intakeSubsystem.hasPiece() == IntakeSubsystem.Possession.NO_PIECE){
                    currentState = AutoState.RETRACT_AND_TURN_1;
                    stateTimer.reset();
                }
                if (stateTimer.seconds() > 2.0) {
                    currentState = AutoState.RETRACT_AND_TURN_1;
                    stateTimer.reset();
                }
                break;

            case RETRACT_AND_TURN_1:
                intakeServo.setPower(-1);
                extenderTarget = 16;
                armTarget = 100;
                if (!follower.isBusy()) {
                    follower.followPath(paths[3]);
                }
                if (stateTimer.seconds() > 1) {
                    currentState = AutoState.DRIVE_TO_PICKUP_2;
                    stateTimer.reset();
                }
                break;

            case DRIVE_TO_PICKUP_2:
                armTarget = 13.5;
                intakeServo.setPower(1);
                if (!follower.isBusy()) {
                    follower.followPath(paths[3]);
                }
                if (stateTimer.seconds() > 1.25) {
                    currentState = AutoState.GRAB_PIECE_2;
                    stateTimer.reset();
                }
                break;

            case GRAB_PIECE_2:
                intakeServo.setPower(1);
                //armTarget = 13.5;
                extenderTarget = 35;
                if(intakeSubsystem.hasPiece() == IntakeSubsystem.Possession.HAS_PIECE){
                    currentState = AutoState.TURN_TO_SCORE_2;
                    stateTimer.reset();
                }
                if (stateTimer.seconds() > 2) {
                    currentState = AutoState.TURN_TO_SCORE_2;
                    stateTimer.reset();
                }
                break;

            case TURN_TO_SCORE_2:
                intakeServo.setPower(0);
                extenderTarget = 16;
                if (!follower.isBusy()) {
                    follower.followPath(paths[2]);
                }
                armTarget = 100;
                if (stateTimer.seconds() > 1) {
                    currentState = AutoState.EXTEND_ARM_2;
                    stateTimer.reset();
                }
                break;

            case EXTEND_ARM_2:
                armTarget=88;
                extenderTarget = 64;
                if (stateTimer.seconds() > 0.8) {
                    currentState = AutoState.SCORE_PIECE_2;
                    stateTimer.reset();
                }
                break;

            case SCORE_PIECE_2:
                intakeServo.setPower(-1);
                if(intakeSubsystem.hasPiece() == IntakeSubsystem.Possession.NO_PIECE){
                    currentState = AutoState.RETRACT_AND_TURN_2;
                    stateTimer.reset();
                }
                if (stateTimer.seconds() > 2.0) {
                    //currentState = AutoState.RETRACT_FOR_PARK;
                    currentState = AutoState.RETRACT_AND_TURN_2;
                    stateTimer.reset();
                }
                break;

            case RETRACT_AND_TURN_2:
                intakeServo.setPower(0);
                extenderTarget = 16;
                armTarget = 100;
                if (!follower.isBusy()) {
                    follower.followPath(paths[4]);
                }
                if (stateTimer.seconds() > 1.0) {
                    currentState = AutoState.DRIVE_TO_PICKUP_3;
                    stateTimer.reset();
                }
                break;

            case DRIVE_TO_PICKUP_3:
                armTarget = 25;
                intakeServo.setPower(-1);
                if (!follower.isBusy()) {
                    follower.followPath(paths[4]);
                }
                if (stateTimer.seconds() > 1) {
                    currentState = AutoState.GRAB_PIECE_3;
                    stateTimer.reset();
                }
                break;

            case GRAB_PIECE_3:
                intakeServo.setPower(1);
                armTarget = 14.5;
                extenderTarget = 40;
                if(intakeSubsystem.hasPiece() == IntakeSubsystem.Possession.HAS_PIECE){
                    currentState = AutoState.TURN_TO_SCORE_3;
                    stateTimer.reset();
                }
                if (stateTimer.seconds() > 2) {
                    currentState = AutoState.TURN_TO_SCORE_3;
                    stateTimer.reset();
                }
                break;

            case TURN_TO_SCORE_3:
                intakeServo.setPower(0);
                extenderTarget = 16;
                if (!follower.isBusy()) {
                    follower.followPath(paths[2]);
                }
                armTarget = 100;
                if (stateTimer.seconds() > 1.25) {
                    currentState = AutoState.EXTEND_ARM_3;
                    stateTimer.reset();
                }
                break;

            case EXTEND_ARM_3:
                armTarget = 88;
                extenderTarget = 64;
                if (stateTimer.seconds() > 0.8) {
                    currentState = AutoState.SCORE_PIECE_3;
                    stateTimer.reset();
                }
                break;

            case SCORE_PIECE_3:
                intakeServo.setPower(-1);
                if(intakeSubsystem.hasPiece() == IntakeSubsystem.Possession.NO_PIECE){
                    currentState = AutoState.RETRACT_FOR_PARK;
                    stateTimer.reset();
                }
                if (stateTimer.seconds() > 1.5) {
                    currentState = AutoState.RETRACT_FOR_PARK;
                    stateTimer.reset();
                }
                break;

            case RETRACT_FOR_PARK:
                //extenderTarget = 16;
                armTarget = 95;
                intakeServo.setPower(0);
                if (!follower.isBusy()) {
                    follower.followPath(paths[5]);
                }
                if (stateTimer.seconds() > 1.0) {
                    currentState = AutoState.PARK;
                    stateTimer.reset();
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    follower.followPath(paths[5]);
                }
                armTarget = 75;
                extenderTarget = 16;
                if (stateTimer.seconds() > 4.0) {
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                requestOpModeStop();
                break;
        }

        // Update follower
        follower.update();
    }

    private void initHardware() {
        follower = new Follower(hardwareMap);
        armMotorRight = hardwareMap.get(DcMotor.class, "BigArmRight");
        armMotorLeft = hardwareMap.get(DcMotor.class, "BigArmLeft");
        extenderLeft = hardwareMap.get(DcMotor.class, "ExtenderLeft");
        extenderRight = hardwareMap.get(DcMotor.class, "ExtenderRight");
        intakeServo = hardwareMap.get(CRServo.class, "IS");
        extenderLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initPID() {
        IntP.add(-33, pid1[0]); IntP.add(180, pid2[0]);
        IntI.add(-33, pid1[1]); IntI.add(180, pid2[1]);
        IntD.add(-33, pid1[2]); IntD.add(180, pid2[2]);
        IntF.add(-33, pid1[3]); IntF.add(180, pid2[3]);

        IntP.createLUT(); IntI.createLUT(); IntD.createLUT(); IntF.createLUT();

        armPid = new PIDFController(IntP.get(0), IntI.get(0), IntD.get(0), IntF.get(0));
        extenderPid = new PIDController(pide[0], pide[1], pide[2]);
    }

    private void updateSensors() {
        double armDeg = ((-armMotorRight.getCurrentPosition() * TICKS_PER_DEGREE) % 360);
        double extenderPositionTicks = -(extenderLeft.getCurrentPosition() - extenderRight.getCurrentPosition()) / 2.0;
        double extenderInches = 16 + extenderPositionTicks * EXTENDER_TICKS;

        // Update PID controllers
        armPid.setPIDF(IntP.get(armDeg), IntI.get(armDeg), IntD.get(armDeg), IntF.get(armDeg));
        double armOutput = -armPid.calculate(armDeg, armTarget);
        double extenderOutput = -extenderPid.calculate(extenderInches, extenderTarget);

        setArmPower(armOutput);
        setExtenderPower(extenderOutput);
    }

    private void setArmPower(double power) {
        armMotorLeft.setPower(-power * power_mult);
        armMotorRight.setPower(power * power_mult);
    }

    private void setExtenderPower(double power) {
        extenderLeft.setPower(power);
        extenderRight.setPower(-power);
    }

    private void updateActuators() {
        // Handle safety limits
        double armDeg = ((-armMotorRight.getCurrentPosition() * TICKS_PER_DEGREE) % 360);
        if ((-33 <= armDeg) && (armDeg <= 70)) {
            if (extenderTarget > 40) extenderTarget = 40;
        } else if ((armDeg > 70) && (armDeg < 120)) {
            if (extenderTarget > 54) extenderTarget = 54;
        } else {
            if (extenderTarget > 30) extenderTarget = 30;
        }
    }
}