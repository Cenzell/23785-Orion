package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Locale;



@Config
public class MotionSubsystem {

    // Hardware Components
    public ServoImplEx HoriExtR;      // Right horizontal extension servo
    public ServoImplEx HoriExtL;      // Left horizontal extension servo
    public ServoImplEx MiniExt;       // Mini extension servo
    public ServoImplEx Wrist;         // Wrist servo
    public ServoImplEx Claw;          // Claw servo
    public ServoImplEx IntakeFlip;    // Intake flip servo
    public DistanceSensor sensorDistance; // sensor
    public DcMotorEx VertRight;       // Right vertical extension motor
    public DcMotorEx MiniArm;         // Mini arm motor
    public DcMotorImpl Intake;          // Intake motor
    public DcMotorEx VertLeft;        // Left vertical extension motor

    // Control Components
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public double vertTicks;
    public double vertIN;
    public static double VertTarget = 0;

    // Motion Control
    public static PIDFController vertExtension;  // Vertical extension PID controller
    public static PIDFController miniArmPID;     // Mini arm PID controller
    public static PIDFController wristPID;

    // State Variables
    private Double armDeg;            // Current arm angle in degrees
    private Double ForwardLimit;      // Forward movement limit
    private Double VertLimit;         // Vertical movement limit
    private long outTakeStartTime = 0;
    private boolean isOutTaking = false;
    private static final double TICKS_PER_DEGREE = 8192.0/360.0;  //encoder ticks per degree
    private double targetAngleDegreesMiniArm;
    public static double wristTarget;
    public static double MAKP = .05;
    public static double MAKI = .002;
    public static double MAKD = .001;
    public static double MAKF = .0001;

    private ElapsedTime transferTimerOne = new ElapsedTime();
    private boolean transferTimerStartedOne = false;
    private ElapsedTime transferTimerTwo = new ElapsedTime();
    private boolean transferTimerStartedTwo = false;
    private ElapsedTime transferTimerThree = new ElapsedTime();
    private boolean transferTimerStartedThree = false;
    private ElapsedTime transferTimerFour = new ElapsedTime();
    private boolean transferTimerStartedFour = false;
    private ElapsedTime transferTimerFive = new ElapsedTime();
    private boolean transferTimerStartedFive = false;


    public static double VertP = 0.35, VertI = 0.25, VertD = 0, VertF = 0;

    public String state = "";
    /**
     * Constructor for Motion Subsystem
     */
    public MotionSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Initialize all hardware components and controllers
     */
    public void init() {
        // Initialize Servos
        HoriExtL = (ServoImplEx) hardwareMap.get(Servo.class, "HoriExtR");
        HoriExtR = (ServoImplEx) hardwareMap.get(Servo.class, "HoriExtL");
        MiniExt = (ServoImplEx) hardwareMap.get(Servo.class, "MiniExt");
        Wrist = (ServoImplEx) hardwareMap.get(Servo.class, "Wrist");
        Claw = (ServoImplEx) hardwareMap.get(Servo.class, "Claw");
        IntakeFlip = (ServoImplEx) hardwareMap.get(Servo.class, "IntakeFlip");

        // Initialize Motors
        VertRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "VertRight");
        MiniArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "MiniArm");
        Intake = (DcMotorImpl) hardwareMap.get(DcMotor.class, "Intake");
        VertLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "VertLeft");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        // Initialize Controllers
        vertExtension = new PIDFController(VertP, VertI, VertD, VertF);
        vertExtension.setTolerance(2);
        miniArmPID = new PIDFController(MAKP,MAKI,MAKD,MAKF);
        miniArmPID.setTolerance(4);

        Wrist.setDirection(Servo.Direction.REVERSE);
        //MiniExt.setDirection(Servo.Direction.REVERSE);
    }

    public void start(){
        drivePos();
    }

    /**
     * Main control loop
     */
    public void loop() {
        // Gamepad 1 Controls
        handleGamepad1Controls();
        // Gamepad 2 Controls
        handleGamepad2Controls();
        // set vertical angle
        updateVert();
        // update arm angle
        setMiniArmAngle();
        // update telemetry
        updateTelemetry();

        if(transferTimerTwo.seconds() >= 1.75 && transferTimerOne.seconds() >= 1.75 && transferTimerThree.seconds() >= 1.75 && transferTimerFour.seconds() >= 1.75 && transferTimerFive.seconds() >= 1.75){
            transferTimerStartedTwo = false;
            transferTimerStartedOne = false;
            transferTimerStartedThree = false;
            transferTimerStartedFour = false;
            transferTimerStartedFive = false;
            transferTimerTwo.reset();
            transferTimerOne.reset();
            transferTimerThree.reset();
            transferTimerFour.reset();
            transferTimerFive.reset();
        }
    }

    /**
     * Handle Gamepad 1 controls for primary functions
     */
    private void handleGamepad1Controls() {
        if (gamepad1.a) intakePrep();
        if (gamepad1.b) outTake();
        if (gamepad1.right_bumper) drivePos();
        if (gamepad1.right_trigger > .05) intakePos();
        if (gamepad1.left_trigger > .05) halfIntakePos();
        if (gamepad1.x) wallPickupPrep();
        if (gamepad1.y) closeClaw();
        if (gamepad1.left_bumper) clawOpen();
        if (gamepad1.dpad_left) wristTarget++;
        if (gamepad1.dpad_right) wristTarget--;
    }

    /**
     * Handle Gamepad 2 controls for secondary functions
     */
    private void handleGamepad2Controls() {
        if (gamepad2.x) specimenPrep();
        if (gamepad2.y) specimenScore();
        if (gamepad2.b) sampleScoreHigh();
        if (gamepad2.a) sampleScoreLow();
        if (gamepad2.left_trigger > .05) transfer();
        if (gamepad2.left_bumper) VertTarget = 4.0;
        if (gamepad2.dpad_left) Wrist.setPosition(Wrist.getPosition()+0.005);
        if (gamepad2.dpad_right) Wrist.setPosition(Wrist.getPosition()-0.005);
        if (gamepad2.dpad_up) IntakeFlip.setPosition(IntakeFlip.getPosition() + 0.005);
        if (gamepad2.dpad_down) IntakeFlip.setPosition(IntakeFlip.getPosition() - 0.005);
        if (gamepad2.back) targetAngleDegreesMiniArm -= 0.05;
        if (gamepad2.start) targetAngleDegreesMiniArm += 0.05;
        if (gamepad2.left_stick_button) MiniExt.setPosition(MiniExt.getPosition() - 0.0005);
        if (gamepad2.right_stick_button) MiniExt.setPosition(MiniExt.getPosition() + 0.0005);
        //if (gamepad2.left_bumper) climbPrep();
        //if (gamepad2.right_bumper) climbOne();
        if (gamepad2.right_bumper) sensorTest();
        //if (gamepad2.right_trigger > .05) climbTwo();
    }

    /**
     * State Functions
     */
    public void intakePrep(){
        IntakeFlip.setPosition(.85);
        Intake.setPower(1);
        state = "Intake Position";

    }

    public void outTake(){
        if (!isOutTaking) {
            IntakeFlip.setPosition(.65);
            outTakeStartTime = System.currentTimeMillis();
            isOutTaking = true;
        } else {
            if (System.currentTimeMillis() - outTakeStartTime >= 250) {
                Intake.setPower(-1);
                isOutTaking = false;
            }
        }
        state = "Out Taking Piece";
    }

    public void drivePos(){
        HoriExtL.setPosition(0.15);
        HoriExtR.setPosition(0.15);
        IntakeFlip.setPosition(.08);
        Intake.setPower(0);
        MiniExt.setPosition(0);
        targetAngleDegreesMiniArm = 17.5;
        Wrist.setPosition(0.7);
        VertTarget = 0;
        state = "Drive Position";

    }

    public void sensorTest() {
        if (sensorDistance.getDistance(DistanceUnit.CM) <= 2.9) {
            Intake.setPower(-.75);
        }
        if (sensorDistance.getDistance(DistanceUnit.CM) >= 3) {
            Intake.setPower(0);
        }
        state = "Sensor Test";
    }
    public void intakePos(){
        HoriExtL.setPosition(.9);
        HoriExtR.setPosition(.9);
        state = "Full Intake Extension";
    }

    public void halfIntakePos(){
        HoriExtL.setPosition(.5);
        HoriExtR.setPosition(.5);
        state = "Half Intake Extension";
    }

    public void wallPickupPrep(){
        clawOpen();
        MiniExt.setPosition(.3);
        targetAngleDegreesMiniArm = 12;
        Wrist.setPosition(0.43);
        state = "Wall Pickup Prep";
    }

    public void transfer() {
        HoriExtL.setPosition(0.15);
        HoriExtR.setPosition(0.15);
        IntakeFlip.setPosition(.44);

        Wrist.setPosition(.47);
        MiniExt.setPosition(.14);

        targetAngleDegreesMiniArm = 16;


        // Continuously check distance sensor regardless of timer state
        double currentDistance = sensorDistance.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance", currentDistance);

        if (currentDistance >= 2.85) {
            Intake.setPower(0);
            closeClaw();
        } else {
            Intake.setPower(-1);
        }

        if (Claw.getPosition() < .1) {
            //VertTarget = 4.0;
            state = "Transfer Position - Vertical Extension";
            transferTimerStartedFour = false;

        }

        if (!transferTimerStartedOne) {
            transferTimerOne.reset();
            transferTimerStartedOne = true;

            if (!transferTimerStartedTwo) {
                transferTimerStartedTwo = true;
                transferTimerTwo.reset();
            }
            if (!transferTimerStartedThree) {
                transferTimerStartedThree = true;
                transferTimerThree.reset();
            }
            if (!transferTimerStartedFour) {
                transferTimerStartedFour = true;
                transferTimerFour.reset();
            }
            if (!transferTimerStartedFive) {
                transferTimerStartedFive = true;
                transferTimerFive.reset();
            }
        }

        /*if (transferTimerOne.seconds() >= 0.5 && !(transferTimerTwo.seconds() >= .825)) {
            transferTimerStartedOne = false;
            state = "Transfer Position - Transfer Started";
        }

        if (transferTimerTwo.seconds() >= .825) {
            Wrist.setPosition(.43);
            MiniExt.setPosition(.27);
            targetAngleDegreesMiniArm = 17.5;
            transferTimerStartedTwo = false;
            state = "Transfer Position - Transfer Almost";
        }

        if (transferTimerThree.seconds() >= 1.5) {
            Claw.setPosition(0);
            state = "Transfer Position - Transfer Complete";
            transferTimerStartedThree = false;
        }
        */

    }

    public void sampleScoreLow(){
        MiniExt.setPosition(1);
        targetAngleDegreesMiniArm = 125;
        Wrist.setPosition(0.6);
        VertTarget = 5;
        state = "Score Sample Low";
    }

    public void setVertTarget(double target){
        VertTarget = target;
    }

    public void sampleScoreHigh(){
        targetAngleDegreesMiniArm = 125;
        Wrist.setPosition(0.6);
        VertTarget = 17;
        MiniExt.setPosition(1);
        state = "Score Sample High";
    }

    public void specimenPrep(){
        VertTarget = 0.0;
        targetAngleDegreesMiniArm = 92;
        Wrist.setPosition(1);
        MiniExt.setPosition(.2);
        state = "Specimen Prep";
    }

    public void specimenScore(){
        VertTarget = 7.5;

        if (vertIN > 6.0){
            clawOpen();
            drivePos();
            state = "Score Specimen Release";
        } else {
            state = "Score Specimen Up";
        }
    }

    public void climbPrep(){
        //Todo set vertical extension
        state = "Climb Prep";
    }

    public void climbOne(){
        //Todo set vertical extension
        //TODO set smth with miniArm position
        state = "Stage One Climb";
    }

    public void climbTwo(){
        //Todo set vertical extension
        state = "Stage Two Climb";
    }

    /**
     * Control Code
     *
     */
    private void extendArm(double x){
        VertLeft.setPower(-x);
        VertRight.setPower(x);
    }

    public void closeClaw(){
        Claw.setPosition(.3);
        state = "Claw Closed";
    }

    public void clawOpen(){
        Claw.setPosition(1);
        state = "Claw Open";
    }

    /**
     * Updates telemetry data
     */
    private void updateTelemetry() {
        telemetry.addData("State", state);
        //telemetry.addData("Right Pos", HoriExtR.getPosition());
        //telemetry.addData("Left Pos", HoriExtL.getPosition());
        telemetry.addData("MiniArm", MiniArm.getCurrentPosition() / TICKS_PER_DEGREE);
        telemetry.addData("miniTarget", targetAngleDegreesMiniArm);
        //telemetry.addData("VertExtL", VertLeft.getCurrentPosition());
        //telemetry.addData("VertExtR", VertRight.getCurrentPosition());
        telemetry.addData("VertTarget", VertTarget);
        //telemetry.addData("WristTarget", wristTarget);
        //telemetry.addData("MiniArm Error", miniArmPID.getPositionError());
        telemetry.addData("vert error", vertExtension.getPositionError());

        telemetry.addData("Vert check",vertExtension.atSetPoint());
        telemetry.addData("MiniArm check", miniArmPID.atSetPoint());

        telemetry.addData("MiniExt Target", MiniExt.getPosition());
        telemetry.addData("WristPos", Wrist.getPosition());
        telemetry.addData("Intake", IntakeFlip.getPosition());
        telemetry.addData("VertIn", vertIN);
        telemetry.addData("Distance (CM)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
    }

    /**
     * PID Control code
     */
    public void updateVert(){
        vertExtension.setPIDF(VertP, VertI, VertD, VertF);

        vertTicks = (VertLeft.getCurrentPosition() + -VertRight.getCurrentPosition())/2.0;
        vertIN = vertTicks / 450;

        double output = vertExtension.calculate(vertIN, VertTarget);
        extendArm(output);
    }

    public void setMiniArmAngle() {
        double currentPosition = MiniArm.getCurrentPosition() / TICKS_PER_DEGREE;
        double pidOutput = miniArmPID.calculate(currentPosition, targetAngleDegreesMiniArm);

        // Apply power limits
        pidOutput = Math.min(Math.max(pidOutput, -1.0), 1.0);

        MiniArm.setPower(-pidOutput);
    }
}