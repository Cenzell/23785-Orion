package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import static java.lang.Math.round;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Motion Subsystem for controlling robot mechanisms including:
 * - Horizontal and vertical extensions
 * - Mini arm and wrist
 * - Intake and claw systems
 * - Climbing mechanisms
 */
@Config
public class MotionSubsystem {

    // Hardware Components
    public ServoImplEx HoriExtR;      // Right horizontal extension servo
    public ServoImplEx HoriExtL;      // Left horizontal extension servo
    public ServoImplEx MiniExt;       // Mini extension servo
    public ServoImplEx Wrist;         // Wrist servo
    public ServoImplEx Claw;          // Claw servo
    public ServoImplEx IntakeFlip;    // Intake flip servo

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
    private double distanceTime = 2000;
    private long startTime;
    private int clawset = 0;
    private long outTakeStartTime = 0;
    private boolean isOutTaking = false;
    private static final double TICKS_PER_DEGREE = 8192.0/360.0;  //encoder ticks per degree
    private double targetAngleDegreesMiniArm = 0;
    public static double wristTarget = 10;
    public static double MAKP = .020;
    public static double MAKI = .0001;
    public static double MAKD = .001;
    public static double MAKF = .0001;

    public static double WP = .05;
    public static double WI = .0;
    public static double WD = .0;
    public static double WF = .0;

    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transferTimerStarted = false;

    public static double VertP = 0.35, VertI = 0.25, VertD = 0, VertF = 0;
    public static PIDFCoefficients VertPIDF;

    public double WristDeg, WristTicks;


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

        // Initialize Controllers
        VertPIDF = new PIDFCoefficients(0,0,0,0);
        vertExtension = new PIDFController(VertP, VertI, VertD, VertF);
        miniArmPID = new PIDFController(MAKP,MAKI,MAKD,MAKF);
        wristPID = new PIDFController(WP, WI, WD, WF);

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
        // set wrist angle
        setWrist();
        // update telemetry
        updateTelemetry();
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

        //if (gamepad1.dpad_right) {wristTarget = -25;}

        //if(gamepad1.dpad_down) extendArm(-1);
        //else if (gamepad1.dpad_up) extendArm(1);
        //else extendArm(0);
    }

    public void updateVert(){
        vertExtension.setPIDF(VertP, VertI, VertD, VertF);

        vertTicks = (VertLeft.getCurrentPosition() + -VertRight.getCurrentPosition())/2.0;
        vertIN = vertTicks / 450;

        double output = vertExtension.calculate(vertIN, VertTarget);
        extendArm(output);

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
        //if (gamepad2.left_bumper) climbPrep();
        //if (gamepad2.right_bumper) climbOne();
        //if (gamepad2.right_trigger > .05) climbTwo();
    }

    /**
     * Control Functions
     */
    public void intakePrep(){
        IntakeFlip.setPosition(.8);
        Intake.setPower(1);
        telemetry.addData("State", "Intake Position");

    }

    public void outTake(){
        if (!isOutTaking) {
            IntakeFlip.setPosition(.7);
            outTakeStartTime = System.currentTimeMillis();
            isOutTaking = true;
        } else {
            if (System.currentTimeMillis() - outTakeStartTime >= 250) {
                Intake.setPower(-1);
                isOutTaking = false;
            }
        }
        telemetry.addData("State", "Out Taking piece");
    }

    public void drivePos(){
        HoriExtL.setPosition(0.15);
        HoriExtR.setPosition(0.15);
        IntakeFlip.setPosition(.08);
        Intake.setPower(0);
        MiniExt.setPosition(.70);
        targetAngleDegreesMiniArm = 20;
        wristTarget = 40;
        VertTarget = 0;
        telemetry.addData("State", "drive Position");

    }

    public void intakePos(){
        HoriExtL.setPosition(.9);
        HoriExtR.setPosition(.9);
        telemetry.addData("State", "full Intake Position");
    }

    public void halfIntakePos(){
        HoriExtL.setPosition(.5);
        HoriExtR.setPosition(.5);
        telemetry.addData("State", "half Intake Position");
    }

    public void wallPickupPrep(){
        clawOpen();
        MiniExt.setPosition(1);
        targetAngleDegreesMiniArm = 21;
        wristTarget = -10;
        telemetry.addData("State", "wall Pickup Prep");
    }

    public void closeClaw(){
        Claw.setPosition(.2);
        telemetry.addData("State", "Claw Close");
    }

    public void clawOpen(){
        Claw.setPosition(1);
        telemetry.addData("State", "Claw Open");
    }

    public void transfer(){
        targetAngleDegreesMiniArm = 25;
        wristTarget = -45;
        Claw.setPosition(1);
        IntakeFlip.setPosition(.25);
        MiniExt.setPosition(0);
        if(miniArmPID.atSetPoint() && wristPID.atSetPoint()) {
            Intake.setPower(-1);

            if (!transferTimerStarted) {
                transferTimer.reset();
                transferTimerStarted = true;
            }

            if (transferTimer.seconds() >= 0.75) { // 3/4 of a second
                Claw.setPosition(0);
                transferTimerStarted = false;
            }
        } else {
            transferTimerStarted = false;
        }
        telemetry.addData("State", "transfer Position");
    }



    public void sampleScoreLow(){
        MiniExt.setPosition(1);
        targetAngleDegreesMiniArm = 125;
        wristTarget = 15;
        VertTarget = 5;
        telemetry.addData("State", "scoreSampleLow");
    }

    public void sampleScoreHigh(){
        targetAngleDegreesMiniArm = 125;
        wristTarget = 15;
        VertTarget = 17;
        MiniExt.setPosition(1);
        telemetry.addData("State", "scoreSampleHigh");
    }

    public void specimenPrep(){
        targetAngleDegreesMiniArm = 105;
        wristTarget = 100;
        VertTarget = 0;
        MiniExt.setPosition(1);
        telemetry.addData("State", "SpecimenPrep");
    }

    public void specimenScore(){
        VertTarget = 12;
        if (vertIN > 10){
            clawOpen();
        }
        telemetry.addData("State", "scoreSpecimen");
    }

    public void climbPrep(){
        //Todo set vertical extension
    }

    public void climbOne(){
        //Todo set vertical extension
        //TODO set smth with miniArm position
    }

    public void climbTwo(){
        //Todo set vertical extension
    }

    public void setMiniArmAngle() {
        double currentPosition = MiniArm.getCurrentPosition() / TICKS_PER_DEGREE;
        double pidOutput = miniArmPID.calculate(currentPosition, targetAngleDegreesMiniArm);

        // Apply power limits
        pidOutput = Math.min(Math.max(pidOutput, -1.0), 1.0);

        MiniArm.setPower(-pidOutput);
    }

    private void extendArm(double x){
        VertLeft.setPower(-x);
        VertRight.setPower(x);
    }

    /**
     * Updates telemetry data
     */
    private void updateTelemetry() {
        telemetry.addData("Right Pos", HoriExtR.getPosition());
        telemetry.addData("Left Pos", HoriExtL.getPosition());
        telemetry.addData("MiniArm", MiniArm.getCurrentPosition() / TICKS_PER_DEGREE);
        telemetry.addData("miniArmTicks", MiniArm.getCurrentPosition());
        telemetry.addData("miniTarget", targetAngleDegreesMiniArm);
        telemetry.addData("VertExtL", VertLeft.getCurrentPosition());
        telemetry.addData("VertExtR", VertRight.getCurrentPosition());
        telemetry.addData("VertTicks", vertTicks);
        //telemetry.addData("VertIN", vertIN);
        telemetry.addData("VertTarget", VertTarget);
        //telemetry.addData("P", vertExtension.getP());
        telemetry.addData("Output", vertExtension.calculate(vertIN, VertTarget));
        telemetry.addData("Target", VertTarget);
        telemetry.addData("WristTicks", WristTicks);
        telemetry.addData("WristDeg", WristDeg);
        telemetry.addData("WristTarget", wristTarget);

    }

    public void setWrist(){
        double currentPosition = Intake.getCurrentPosition() / (TICKS_PER_DEGREE * 4);
        double pidOutput = wristPID.calculate(currentPosition, wristTarget);


        // Apply power limits
        pidOutput = ((Math.min(Math.max(pidOutput, -1.0), 1.0))/2.0) + .5;

        Wrist.setPosition(pidOutput);
    }

    public void Limits(){
        //TODO: Find good limits and enable
        if(armDeg > 999){
            armDeg = 45.0;
        } else if (armDeg < -100){
            armDeg = 45.0;
        }
    }

}
