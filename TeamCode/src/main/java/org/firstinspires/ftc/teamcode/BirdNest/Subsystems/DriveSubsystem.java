package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import pedroPathing.constants.*;

@Config
public class DriveSubsystem {

    private HardwareMap hardwareMap;
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU gyro;
    private Telemetry telemetry;
    private Gamepad gamepad1, gamepad2;

    FConstants fConstants;
    LConstants lConstants;

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    public DriveSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize motors
        frontLeft = new MotorEx(hardwareMap, "FrontLeft");
        frontRight = new MotorEx(hardwareMap, "FrontRight");
        backLeft = new MotorEx(hardwareMap, "BackLeft");
        backRight = new MotorEx(hardwareMap, "BackRight");

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        // Update follower with gamepad inputs - added false parameter for field-centric drive
        follower.setTeleOpMovementVectors(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false  // Set to false for field-centric drive
        );
        follower.update();

        // Add telemetry
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
    }
}