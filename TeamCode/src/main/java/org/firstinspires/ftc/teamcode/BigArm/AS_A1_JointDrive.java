package org.firstinspires.ftc.teamcode.BigArm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Config
@TeleOp(name="AS_A1_Joint")
public class AS_A1_JointDrive  extends OpMode {

    MotorEx frontLeft, frontRight, backLeft, backRight;
    Motor.Encoder encoderLeft, encoderRight, encoderAux;
    IMU gyro;

    public static double TICKS_TO_INCHES = 1/(8192 / ((35 / 25.4) * Math.PI));

    public static double TRACKWIDTH = 5.7;
    public static double CENTER_WHEEL_OFFSET = 4.1333;

    HolonomicOdometry holomonic;
    OdometrySubsystem odometry;
    MecanumDrive mecanum;

    //HardwareMap hardwareMap;

    InterpLUT IntP = new InterpLUT(), IntI = new InterpLUT(), IntD = new InterpLUT(), IntF = new InterpLUT();

    DcMotor armMotorLeft;
    DcMotor armMotorRight;
    DcMotor extenderLeft, extenderRight; //TODO Check 560 ticks per rev
    CRServo intakeServo;

    //ServoEx intakeServo;
    //TODO Limit C=42/cos(x)

    PIDFController Pid;

    boolean debugPID = false;

    //Set in dashboar
    public static double[] pid1 = new double[]{0,0,0,0};
    public static double[] pid2 = new double[]{0,0,0,0};
    public static double ff;
    public static double power_mult;
    public static double target;

    double TICKS_PER_DEGREE = 360.0/8192.0;

    @Override
    public void init() {

        frontLeft = new MotorEx(hardwareMap, "FrontLeft");
        frontRight = new MotorEx(hardwareMap, "FrontRight");
        backLeft = new MotorEx(hardwareMap, "BackLeft");
        backRight = new MotorEx(hardwareMap, "BackRight");

        encoderLeft = frontLeft.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight = frontRight.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        encoderAux = backLeft.encoder.setDistancePerPulse(TICKS_TO_INCHES);

        holomonic = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderAux::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        odometry = new OdometrySubsystem(holomonic);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotorRight = hardwareMap.get(DcMotor.class, "BigArmRight");
        armMotorLeft = hardwareMap.get(DcMotor.class, "BigArmLeft");

        extenderLeft = hardwareMap.get(DcMotor.class, "ExtenderLeft");
        extenderRight = hardwareMap.get(DcMotor.class, "ExtenderRight");

        intakeServo = hardwareMap.get(CRServo.class, "IS");

        //TODO - Not sure which one needs to be reversed.
        //extenderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //extenderRight.setDirection(DcMotorSimple.Direction.REVERSE);

        power_mult = 0.85;

        pid1 = new double[]{0.015, 0, 0.0000005, 0.009};
        pid2 = new double[]{0.015, 0, 0.0000005, 0.009};

//        IntP.add(-55, pid1[0]);
//        IntP.add(365, pid1[0]);
//        IntI.add(-55,pid1[1]);
//        IntI.add(365, pid1[1]);
//        IntD.add(-55, pid1[2]);
//        IntD.add(365, pid1[2]);
//        IntF.add(-55,pid1[3]);
//        IntF.add(365, pid1[3]);

        IntP.add(-55, pid1[0]);
        IntP.add(365, pid2[0]);
        IntI.add(-55,pid1[1]);
        IntI.add(365, pid2[1]);
        IntD.add(-55, pid1[2]);
        IntD.add(365, pid2[2]);
        IntF.add(-55,pid1[3]);
        IntF.add(365, pid2[3]);

        IntP.createLUT(); IntI.createLUT(); IntD.createLUT(); IntF.createLUT();

        ff = 0; target = 0;

        Pid = new PIDFController(0,0,0,0);
    }

    @Override
    public void loop() {
        FtcDashboard.getInstance().updateConfig();

        double armPos = -armMotorRight.getCurrentPosition();
        double armDeg = ((-armMotorRight.getCurrentPosition() * TICKS_PER_DEGREE) % 360);

//        Pid.setPIDF(
//                IntP.get(armDeg),
//                IntI.get(armDeg),
//                IntD.get(armDeg),
//                IntF.get(armDeg));

        Pid.setPIDF(pid1[0],pid1[1],pid1[2],pid1[3]);

        //TODO Add extension and get a extension PID - Kinda my whole idea behind using interpolation.

        if(gamepad1.left_bumper){
            intakeServo.setPower(-1);
        } else if (gamepad1.right_bumper) {
            intakeServo.setPower(1);
        } else {
            intakeServo.setPower(0);
            //intakeServo.disable();
        }

        if(gamepad1.dpad_left){
            target = target - 1.8;
        } if(gamepad1.dpad_right){
            target = target + 1.8;
        }

//        if(gamepad1.a){ //Arm with Dashboard PID
//            double output = -Pid.calculate(armDeg, target); //Will be pid1 setpoint
//            arm(output);
//        }else if (gamepad1.y){
//            double output = Pid.calculate(armPos,0); //pid[x] setpoint //TODO Use to get more pid value ranges
//            arm(output);
//        } else if (gamepad1.b){
//            //armMotor.setPower(IntF.get(armMotor.getCurrentPosition()));
//            arm(-Math.cos(Math.toRadians(target/TICKS_PER_DEGREE)) * ff);
//        } else if (gamepad1.x){
//            IntP.createLUT(); IntI. createLUT(); IntD.createLUT(); IntF.createLUT();
//        } else { //Not needed but to ensure that if there is no input the power is zero.
//            arm(0);
//        }

        arm(-Pid.calculate(armDeg, target));

        if(gamepad1.dpad_up){
            extender(0.70);
        } else if (gamepad1.dpad_down) {
            extender(-0.70);
        } else {
            extender(0);
        }

        FtcDashboard.getInstance().updateConfig();

        telemetry.addData("Encoder Pos:", armPos);
        telemetry.addData("Setpoint:", target);
        telemetry.addData("Deg", armDeg);
        telemetry.addData("Output:", Pid.calculate(armPos,target));

        if(debugPID){
            telemetry.addData("IntP:", IntP.get(armDeg));
            telemetry.addData("IntI:", IntI.get(armDeg));
            telemetry.addData("IntD:", IntD.get(armDeg));
            telemetry.addData("IntF:", IntF.get(armDeg));

            telemetry.addData("P:", Pid.getP());
            telemetry.addData("I:", Pid.getI());
            telemetry.addData("D:", Pid.getD());
            telemetry.addData("F:", Pid.getF());
        }

        holomonic.updatePose();
        mecanum.driveFieldCentric(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, odometry.getPose().getHeading());
        //mecanum.driveFieldCentric(0.0, -0.0, 0.0, 0.0);
        if(gamepad1.a){
            holomonic.rotatePose(0);
        }

        telemetry.addLine("/// Odometry ///");
        telemetry.addData("Heading:", Math.toDegrees(odometry.getPose().getHeading()));
        telemetry.addData("x", odometry.getPose().getX());
        telemetry.addData("Heading (IMU):", gyro.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Pose:", odometry.getPose().getX());
        telemetry.update();

        //extender(0);
    }

    public void extender(double power){
        extenderLeft.setPower(-power*power_mult);
        extenderRight.setPower(-power);
    }

    public void arm(double power){
        armMotorLeft.setPower(-power);
        armMotorRight.setPower(power);
    }
}