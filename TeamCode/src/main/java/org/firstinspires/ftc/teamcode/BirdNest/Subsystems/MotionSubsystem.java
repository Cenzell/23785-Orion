package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private long outTakeStartTime = 0;
    private boolean isOutTaking = false;
    private static final double TICKS_PER_DEGREE = 8192.0/360.0;  //encoder ticks per degree
    private double targetAngleDegreesMiniArm;
    public static double wristTarget;
    public static double MAKP = .020;
    public static double MAKI = .001;
    public static double MAKD = .001;
    public static double MAKF = .0001;

    public static double WP = .1;
    public static double WI = .0;
    public static double WD = .0;
    public static double WF = .0;

    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transferTimerStarted = false;

    public static double VertP = 0.35, VertI = 0.25, VertD = 0, VertF = 0;
    public static PIDFCoefficients VertPIDF;

    public double WristDeg, WristTicks;

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

        // Initialize Controllers
        //VertPIDF = new PIDFCoefficients(0,0,0,0);
        vertExtension = new PIDFController(VertP, VertI, VertD, VertF);
        //vertExtension.setTolerance(90);
        miniArmPID = new PIDFController(MAKP,MAKI,MAKD,MAKF);
        miniArmPID.setTolerance(3);
        wristPID = new PIDFController(WP, WI, WD, WF);
        wristPID.setTolerance(3);
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
     * State Functions
     */
    public void intakePrep(){
        IntakeFlip.setPosition(.8);
        Intake.setPower(1);
        state = "Intake Position";

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
        state = "Out Taking Piece";
    }

    public void drivePos(){
        HoriExtL.setPosition(0.15);
        HoriExtR.setPosition(0.15);
        IntakeFlip.setPosition(.08);
        Intake.setPower(0);
        MiniExt.setPosition(.70);
        targetAngleDegreesMiniArm = 25;
        wristTarget = 0;
        VertTarget = 0;
        state = "Drive Position";

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
        MiniExt.setPosition(1);
        targetAngleDegreesMiniArm = 7;
        wristTarget = -23;
        state = "Wall Pickup Prep";
    }

    public void transfer(){
        targetAngleDegreesMiniArm = 25;
        wristTarget = -80;
        Claw.setPosition(1);
        IntakeFlip.setPosition(.25);
        MiniExt.setPosition(0);
        if(miniArmPID.atSetPoint() && wristPID.atSetPoint()) {
            Intake.setPower(-1);

            if (!transferTimerStarted) {
                transferTimer.reset();
                transferTimerStarted = true;
            }

            if (transferTimer.seconds() >= 0.75) {
                Claw.setPosition(0);
                transferTimerStarted = false;
            }
        } else {
            transferTimerStarted = false;
        }
        state = "Transfer Position";
    }

    public void sampleScoreLow(){
        MiniExt.setPosition(1);
        targetAngleDegreesMiniArm = 125;
        wristTarget = 35;
        VertTarget = 5;
        state = "Score Sample Low";
    }

    public void sampleScoreHigh(){
        targetAngleDegreesMiniArm = 125;
        wristTarget = 35;
        VertTarget = 17;
        MiniExt.setPosition(1);
        state = "Score Sample High";
    }

    public void specimenPrep(){
        targetAngleDegreesMiniArm = 105;
        wristTarget = 100;
        VertTarget = 0;
        MiniExt.setPosition(0);
        state = "Specimen Prep";
    }

    public void specimenScore(){
        VertTarget = 8;

        if (vertIN > 5.5){
            clawOpen();
        }
        state = "Score Specimen";
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
        Claw.setPosition(.2);
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
        telemetry.addData("Right Pos", HoriExtR.getPosition());
        telemetry.addData("Left Pos", HoriExtL.getPosition());
        telemetry.addData("MiniArm", MiniArm.getCurrentPosition() / TICKS_PER_DEGREE);
        telemetry.addData("miniTarget", targetAngleDegreesMiniArm);
        telemetry.addData("VertExtL", VertLeft.getCurrentPosition());
        telemetry.addData("VertExtR", VertRight.getCurrentPosition());
        telemetry.addData("VertIn", vertIN);
        telemetry.addData("VertTarget", VertTarget);
        telemetry.addData("WristDeg", Intake.getCurrentPosition() / (TICKS_PER_DEGREE * 4.0));
        telemetry.addData("WristTarget", wristTarget);

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

    public void setWrist(){
        double currentPosition = Intake.getCurrentPosition() / (TICKS_PER_DEGREE * 4.0);
        double pidOutput = wristPID.calculate(currentPosition, wristTarget);

        // Apply power limits
        pidOutput = (pidOutput/2.0) + .5;

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