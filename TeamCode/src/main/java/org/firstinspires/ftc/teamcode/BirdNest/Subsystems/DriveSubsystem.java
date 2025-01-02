package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
public class DriveSubsystem {

    private HardwareMap hardwareMap;

    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU gyro;

    //private ThreeWheelLocalizer localizer;
    private Telemetry telemetry;
    private Gamepad gamepad1, gamepad2;

    FConstants fConstants;
    LConstants lConstants;



    private Follower follower;

    public DriveSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init(){
        // Initialize motors
        frontLeft = new MotorEx(hardwareMap, "FrontLeft");
        frontRight = new MotorEx(hardwareMap, "FrontRight");
        backLeft = new MotorEx(hardwareMap, "BackLeft");
        backRight = new MotorEx(hardwareMap, "BackRight");

        frontRight.setInverted(true);
        backLeft.setInverted(true);
        backLeft.setInverted(true);

        fConstants = new FConstants();
        lConstants = new LConstants();

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Initialize localizer with starting pose at (0,0,0)
        //localizer = new ThreeWheelLocalizer(hardwareMap, new Pose(0, 0, 0));

        // Initialize follower
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.startTeleopDrive();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize IMU with correct orientation
        //gyro = hardwareMap.get(IMU.class, "imu");
        //IMU.Parameters parameters = new IMU.Parameters(
                //new RevHubOrientationOnRobot(
                        //RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        //RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        //gyro.initialize(parameters);
    }

    public void loop(){

        // Update follower with gamepad inputs
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        if(gamepad2.b){
            //gyro.resetYaw();
            //localizer.setPose(new Pose(localizer.getPose().getX(), localizer.getPose().getY(), 0));
        }

        // Add telemetry
        //telemetry.addLine("=== Odometry Data ===");
        //Pose currentPose = localizer.getPose();
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        //telemetry.addData("IMU Heading", gyro.getRobotYawPitchRollAngles().getYaw());
        //telemetry.update();
    }

    //public ThreeWheelLocalizer getLocalizer() {
        //return localizer;
    //}

    //public Pose getCurrentPose() {
        //return localizer.getPose();
    //}
}