package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@Config
//@TeleOp(name="AS_FTCLibDrive")
public class AS_FTCLib_Odo extends OpMode {

    MotorEx frontLeft, frontRight, backLeft, backRight;
    Encoder encoderLeft, encoderRight, encoderAux;
    IMU gyro;

    public static double TICKS_TO_INCHES = 8192 / ((35 / 25.4) * Math.PI);

    public static double TRACKWIDTH = 7.9;
    public static double CENTER_WHEEL_OFFSET = 10;

    HolonomicOdometry holomonic;
    OdometrySubsystem odometry;
    MecanumDrive mecanum;

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

    }

    @Override
    public void loop() {
        holomonic.updatePose();
        mecanum.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, odometry.getPose().getHeading());
        //mecanum.driveFieldCentric(0.0, -0.0, 0.0, 0.0);
        if(gamepad1.a){
            holomonic.rotatePose(0);
        }

        telemetry.addLine("/// Odometry ///");
        telemetry.addData("Heading:", odometry.getPose().getHeading());
        telemetry.addData("Heading (IMU):", gyro.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Pose:", odometry.getPose().getX());
        telemetry.update();
    }
}
