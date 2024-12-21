package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BirdNest.lib.pedroPathing.follower.Follower;

@Config
public class DriveSubsystem {

    HardwareMap hardwareMap;

    MotorEx frontLeft, frontRight, backLeft, backRight;
    Motor.Encoder encoderLeft, encoderRight, encoderAux;
    IMU gyro;

    public static double TICKS_TO_INCHES = 8192 / ((35 / 25.4) * Math.PI);

    public static double TRACKWIDTH = 5.7;
    public static double CENTER_WHEEL_OFFSET = 4.13333;

    HolonomicOdometry holomonic;
    OdometrySubsystem odometry;
    MecanumDrive mecanum;

    Telemetry telemetry;
    Gamepad gamepad1, gamepad2;

    private Follower follower;

    public DriveSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init(){
        follower = new Follower(hardwareMap);

        frontLeft = new MotorEx(hardwareMap, "FrontLeft");
        frontRight = new MotorEx(hardwareMap, "FrontRight");
        backLeft = new MotorEx(hardwareMap, "BackLeft");
        backRight = new MotorEx(hardwareMap, "BackRight");

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        follower.startTeleopDrive();

        encoderLeft = frontLeft.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight = frontRight.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        encoderAux = backLeft.encoder.setDistancePerPulse(TICKS_TO_INCHES);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);
    }

    public void loop(){
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        if(gamepad2.b ){
            //holomonic.rotatePose(0);
            gyro.resetYaw();
        }

        telemetry.addLine("/// Odometry ///");
        //telemetry.addData("Heading:", Math.toDegrees(odometry.getPose().getHeading()));
        //telemetry.addData("x", odometry.getPose().getX());
        telemetry.addData("Heading (IMU):", gyro.getRobotYawPitchRollAngles().getYaw());
        //telemetry.addData("Pose:", odometry.getPose().getX());
        telemetry.update();
    }
}
