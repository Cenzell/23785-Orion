package org.firstinspires.ftc.teamcode.AUTO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous (name = "L-S-3s-P", group = "Orion")
public class Good extends OpMode {

    private Follower follower;

    DcMotor armMotorLeft;
    DcMotor armMotorRight;
    DcMotor extenderLeft, extenderRight; //TODO Check 560 ticks per rev
    //TODO Limit C=42/cos(x)

    InterpLUT IntP = new InterpLUT(), IntI = new InterpLUT(), IntD = new InterpLUT(), IntF = new InterpLUT();
    PIDFController Pid;
    public static PIDController ExtenderPid;

    CRServo intakeServo;

    //Set in dashboard
    public static boolean debugPID = false;
    public static double[] pid1 = new double[]{0,0,0,0};
    public static double[] pid2 = new double[]{0,0,0,0};
    public static double[] pide = new double[]{0.08,0,0};
    public static double ff;
    public static double power_mult;
    public static double target;
    double TICKS_PER_DEGREE = 360.0/8192.0;

    /*TODO*/public static double EXTENDER_TICKS = (10.0 / 257.5)/3; // We want inches per tick. So we will use the motors to extend the arm out 10 inches and then do: 10 / however many ticks we get.
    double extenderPositionTicks = 0;
    public static PIDController extenderPID = new PIDController(0,0,0);
    double extenderInches = 0;
    double extenderTicks = 0;
    double extensionMax = 42;
    double extenderTarget = 48;
    double startTime;
    double tempTime;

    PathBuilder toSub = new PathBuilder();
    PathBuilder toPiece = new PathBuilder();
    PathBuilder toScore = new PathBuilder();
    PathBuilder toPieceTwo = new PathBuilder();
    PathBuilder toPieceThree = new PathBuilder();
    PathBuilder park = new PathBuilder();

    boolean first = true;
    boolean firstTime = true;
    boolean thirdTime = true;
    boolean fourthTime = true;

    double twoTime;

    Timing.Timer timer;

    @Override
    public void init() {

        follower = new Follower(hardwareMap);

        toSub
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(10.000, 60, Point.CARTESIAN),
                                new Point(40, 60, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        toPiece
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(40.000, 60.000, Point.CARTESIAN),
                                new Point(30.500, 112.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        toScore
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(40.000, 60.000, Point.CARTESIAN),
                                new Point(30.500, 112.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        toPieceTwo
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(40.000, 60.000, Point.CARTESIAN),
                                new Point(30.500, 112.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-30));
        toPieceThree
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(40.000, 60.000, Point.CARTESIAN),
                                new Point(30.500, 112.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-45));
        park
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(30.500, 112.500, Point.CARTESIAN),
                                new Point(80.000, 115.000, Point.CARTESIAN),
                                new Point(80.000, 80.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90));

        follower.setPose(new Pose(10.000, 60, 0));

        armMotorRight = hardwareMap.get(DcMotor.class, "BigArmRight");
        armMotorLeft = hardwareMap.get(DcMotor.class, "BigArmLeft");

        extenderLeft = hardwareMap.get(DcMotor.class, "ExtenderLeft");
        extenderRight = hardwareMap.get(DcMotor.class, "ExtenderRight");

        //TODO - Not sure which one needs to be reversed.
        extenderLeft.setDirection(DcMotor.Direction.REVERSE);

        power_mult = 0.85;

        pid1 = new double[]{0.036, 0, 0, 0.0009};
        pid2 = new double[]{0.036, 0, 0, 0.0009};

        extenderPID = new PIDController(0,0,0);

        IntP.add(-33, pid1[0]);
        IntP.add(180, pid2[0]);
        IntI.add(-33,pid1[1]);
        IntI.add(180, pid2[1]);
        IntD.add(-33, pid1[2]);
        IntD.add(180, pid2[2]);
        IntF.add(-33,pid1[3]);
        IntF.add(180, pid2[3]);

        IntP.createLUT(); IntI.createLUT(); IntD.createLUT(); IntF.createLUT();

        ff = 0.1; target = 0;

        Pid = new PIDFController(0,0,0,0);
        Pid.setPIDF(
                IntP.get(0),
                IntI.get(0),
                IntD.get(0),
                IntF.get(0));

        intakeServo = hardwareMap.get(CRServo.class, "IS");

        PathBuilder hold = new PathBuilder();

        hold.addPath(new BezierCurve(new Point(10.000, 84.983, Point.CARTESIAN),
                new Point(30.792, 84.193, Point.CARTESIAN)));

        //follower.followPath(hold.build());

        startTime = System.currentTimeMillis();
        tempTime = System.currentTimeMillis();
        twoTime = System.currentTimeMillis();

        timer = new Timing.Timer(30);
    }

    @Override
    public void loop() {
        //init
        if(first){
            startTime = System.currentTimeMillis();
            tempTime = System.currentTimeMillis() + 150;
            twoTime = System.currentTimeMillis() + 150;
            timer.start();
            first = false;
        }

        //approach
        if(System.currentTimeMillis() < tempTime){
            target = 72;
            extenderTarget = 54;
        } else{
            if(firstTime){
                follower.followPath(toSub.build());
                firstTime = false;
            } else{
                follower.update();
            }
        }

        //score specimen
        if((timer.elapsedTime() > 2.5) && (timer.elapsedTime() <3.5)){
            if(thirdTime){
                extenderTarget = 16;
                thirdTime = false;
            }
        }

        //drive to piece pick up
        if((timer.elapsedTime() > 3.5 && timer.elapsedTime() < 6)){
            target = 25;
            if (fourthTime){
                intakeServo.setPower(-1);
                follower.followPath(toPiece.build());
                fourthTime = false;
            }
        }


        //grab piece
        if (timer.elapsedTime() > 6 && timer.elapsedTime() < 8.5) {
            intakeServo.setPower(1);
            target = 11.5;
            extenderTarget = 33;
        }

        //turn to score
        if(timer.elapsedTime() > 8.5 && timer.elapsedTime() < 11){
            intakeServo.setPower(0);
            extenderTarget = 16;
            follower.followPath(toScore.build());
            target = 80;
        }

        //extend arm
        if(timer.elapsedTime() > 11 && timer.elapsedTime() < 14){
            extenderTarget = 54;
        }

        //score piece
        if(timer.elapsedTime() > 14 && timer.elapsedTime() < 15){
            intakeServo.setPower(1);
        }

        //drop arm and turn to next piece
        if(timer.elapsedTime() > 15 && timer.elapsedTime() < 16){
            intakeServo.setPower(0);
            extenderTarget = 16;
            target = 11.5;
            follower.followPath(toPieceTwo.build());
        }

        //pick piece 2
        if(timer.elapsedTime() > 16 && timer.elapsedTime() < 18){
            intakeServo.setPower(1);
            extenderTarget = 33;
        }

        //turn to score
        if(timer.elapsedTime() > 18 && timer.elapsedTime() < 20.5){
            intakeServo.setPower(0);
            extenderTarget = 16;
            follower.followPath(toScore.build());
            target = 80;
        }

        //extend arm
        if(timer.elapsedTime() > 20.5 && timer.elapsedTime() < 23.5){
            extenderTarget = 54;
        }

        //score piece
        if(timer.elapsedTime() > 23.5 && timer.elapsedTime() < 24.5){
            intakeServo.setPower(1);
        }

        //drop arm and turn to next piece
        if(timer.elapsedTime() > 24.5 && timer.elapsedTime() < 25.5){
            intakeServo.setPower(0);
            extenderTarget = 16;
            target = 11.5;
            follower.followPath(toPieceThree.build());
        }

        //pick piece 2
        if(timer.elapsedTime() > 25.5 && timer.elapsedTime() < 27.5){
            intakeServo.setPower(1);
            extenderTarget = 33;
        }

        //turn to score
        if(timer.elapsedTime() > 18 && timer.elapsedTime() < 20.5){
            intakeServo.setPower(0);
            extenderTarget = 16;
            follower.followPath(toScore.build());
            target = 80;
        }

        //extend arm
        if(timer.elapsedTime() > 20.5 && timer.elapsedTime() < 23.5){
            extenderTarget = 54;
        }

        //score piece
        if(timer.elapsedTime() > 23.5 && timer.elapsedTime() < 24.5){
            intakeServo.setPower(1);
        }

        if(timer.elapsedTime() > 25 && timer.elapsedTime() < 26){
            extenderTarget = 16;
            target = 20;
            intakeServo.setPower(0);
        }

        if(timer.elapsedTime() > 26 && timer.elapsedTime() < 30) {
            follower.followPath(park.build());
            target = 45;
            extenderTarget = 33;
        }




        telemetry.addData("Start", startTime);
        telemetry.addData("Temp", tempTime + 100);
        telemetry.addData("First", first);
        telemetry.addData("TwoTime", twoTime);
        telemetry.addData("FirstTime", firstTime);
        telemetry.addData("ThirdTime", thirdTime);
        telemetry.addData("Extender Inches", extenderInches);
        telemetry.addData("Target Inches", extenderTarget);
        telemetry.addData("Pid Output", -extenderPID.calculate(extenderInches, extenderTarget));
        telemetry.addData("Timer", timer.elapsedTime());

        extenderPID.setPID(pide[0], pide[1], pide[2]);

        double armPos = -armMotorRight.getCurrentPosition();
        double armDeg = ((-armMotorRight.getCurrentPosition() * TICKS_PER_DEGREE) % 360);

        //Get the average ticks - Should help balance off inaccuracy.
        extenderPositionTicks = -(extenderLeft.getCurrentPosition() - extenderRight.getCurrentPosition()) / 2.0;
        extenderInches = 16 + extenderPositionTicks * EXTENDER_TICKS;
        extensionMax = 33;//42-21 * Math.cos(armDeg);

        if ((-33 <= armDeg) && ((armDeg <= 70))){
            extensionMax = 33;
            if(extenderInches > 33){
                extenderTarget = 33;
            }
        } if ((armDeg > 70) && (armDeg < 120)){
            extensionMax = 54;
        } else {
            extensionMax = 30;
            if (extenderTarget > 30){
                extenderTarget = 30;
            }
        }

        extender(-extenderPID.calculate(extenderInches, extenderTarget));

        if(gamepad2.b){
            target = 75;
        }
        if(gamepad2.x){
            extenderTarget = 52;
        }

        if(gamepad2.dpad_right && target >= 118){
            target = target - 1;
        } else if(gamepad2.dpad_left){
            target = target - 1.8;
        } if(gamepad2.dpad_right){
            target = target + 1.8;
        }

        if(extenderTarget >= extensionMax) {
            extenderTarget = extenderTarget - 1.5;
        }else if (extenderTarget <= 14){
            extenderTarget = extenderTarget + 0.1;
        }else if(gamepad2.right_bumper){
            extenderTarget = extenderTarget + 1;
        } else if (gamepad2.left_bumper) {
            extenderTarget = extenderTarget - 1;
        }

        //TODO Test
        Pid.setPIDF(
                IntP.get(armDeg),
                IntI.get(armDeg),
                IntD.get(armDeg),
                IntF.get(armDeg));

        double output = -Pid.calculate(armDeg, target);
        arm(output);

        telemetry.update();
    }
    public void extender(double power){
        extenderLeft.setPower(power);
        extenderRight.setPower(-power);
    }

    public void arm(double power){
        armMotorLeft.setPower(-power);
        armMotorRight.setPower(power);
    }
}