package org.firstinspires.ftc.teamcode.Etc.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="TS_MotorDirTest")
public class TS_MotorDirection extends OpMode {

    MotorEx frontLeft = new MotorEx(hardwareMap, "FrontLeftDrive");
    MotorEx frontRight = new MotorEx(hardwareMap, "FrontRightDrive");
    MotorEx backLeft = new MotorEx(hardwareMap, "BackLeftDrive");
    MotorEx backRight = new MotorEx(hardwareMap, "BackRightDrive");

    MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight,
            backLeft, backRight);

    public static boolean FLInvert = false;
    public static boolean FRInvert = false;
    public static boolean BLInvert = false;
    public static boolean BRInvert = false;

    double testSpeed = 0.75;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        //With the @Config lets the direction of each motor be reversed from FTC Dashboard.
        if (FLInvert) frontLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        if (FRInvert) frontRight.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        if (BLInvert) backLeft.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        if (BRInvert) backRight.motorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        if (gamepad1.dpad_down){
            mecanum.driveWithMotorPowers(-testSpeed,-testSpeed,-testSpeed,-testSpeed);
        } else if (gamepad1.dpad_up){
            mecanum.driveWithMotorPowers(testSpeed,testSpeed,testSpeed,testSpeed);
        } else if (gamepad1.dpad_left){
            mecanum.driveWithMotorPowers(-testSpeed,testSpeed,testSpeed,-testSpeed);
        } else if (gamepad1.dpad_right){
            mecanum.driveWithMotorPowers(testSpeed,-testSpeed,-testSpeed,testSpeed);
        } else {
            mecanum.stop();
        }
    }
}
