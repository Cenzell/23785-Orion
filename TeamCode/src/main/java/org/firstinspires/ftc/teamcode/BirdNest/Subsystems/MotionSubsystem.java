package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.*;
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
    public TouchSensor digitalTouch;

    // Control Components
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public double vertTicks;
    public double vertIN;
    public static double VertTarget = 0.75;

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
    public static double targetAngleDegreesMiniArm;
    public static double wristTarget;
    public static double MAKP = .08;
    public static double MAKI = .002;
    public static double MAKD = .001;
    public static double MAKF = .0001;

    double vertOffest = 0;


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

        //Wrist.setDirection(Servo.Direction.REVERSE);
        //MiniExt.setDirection(Servo.Direction.REVERSE);

        digitalTouch = hardwareMap.get(TouchSensor.class, "sensor_digital");
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
        if (gamepad1.dpad_left) wristTarget = wristTarget + 0.005;
        if (gamepad1.dpad_right) wristTarget = wristTarget - 0.005;
        if (gamepad1.options) Wrist.setPosition(wristTarget);
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
        if (gamepad2.back) setVertTarget(3.0);
        if (gamepad2.start) targetAngleDegreesMiniArm += 0.05;
        if (gamepad2.left_stick_button) MiniExt.setPosition(MiniExt.getPosition() - 0.0005);
        if (gamepad2.right_stick_button) MiniExt.setPosition(MiniExt.getPosition() + 0.0005);
        //if (gamepad2.left_bumper) climbPrep();
        //if (gamepad2.right_bumper) climbOne();
        //if (gamepad2.right_trigger > .05) climbTwo();
    }

    /**
     * State Functions
     */
    public void intakePrep(){
        IntakeFlip.setPosition(.85);
        Intake.setPower(1);
        clawOpen();
        state = "Intake Position";

    }

    public void outTake(){
        if (!isOutTaking) {
            IntakeFlip.setPosition(.85);
            outTakeStartTime = System.currentTimeMillis();
            isOutTaking = true;
        } else {
            if (System.currentTimeMillis() - outTakeStartTime >= 500) {
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
        VertTarget = 0.75;
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
        MiniExt.setPosition(.66);
        targetAngleDegreesMiniArm = 12;
        Wrist.setPosition(0.48);
        state = "Wall Pickup Prep";
    }

    public void transfer() {
        HoriExtL.setPosition(0.15);
        HoriExtR.setPosition(0.15);
        IntakeFlip.setPosition(.475);
        VertTarget = 0;

        Wrist.setPosition(.60);
        MiniExt.setPosition(.175);

        targetAngleDegreesMiniArm = 16.5;

        // Continuously check distance sensor regardless of timer state
        double currentDistance = sensorDistance.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance", currentDistance);

        if (currentDistance >= 3) {
            Intake.setPower(0);

        if(vertExtension.atSetPoint()){
            closeClaw();
        }
        } else {
            Intake.setPower(-1);
        }

        if (Claw.getPosition() < .1) {
            //VertTarget = 4.0;
            state = "Transfer Position - Vertical Extension";
        }

    }

    public void sampleScoreLow(){
        MiniExt.setPosition(1);
        targetAngleDegreesMiniArm = 125;
        Wrist.setPosition(0.92);
        VertTarget = 3.5;
        state = "Score Sample Low";
    }

    public void setVertTarget(double target){
        VertTarget = target;
    }

    public void sampleScoreHigh(){
        targetAngleDegreesMiniArm = 125;
        Wrist.setPosition(0.92);
        VertTarget = 15;
        MiniExt.setPosition(1);
        state = "Score Sample High";
    }

    public void specimenPrep(){
        VertTarget = 0.15;
        targetAngleDegreesMiniArm = 98;
        Wrist.setPosition(1);
        MiniExt.setPosition(.17);
        state = "Specimen Prep";
    }

    public void specimenScore(){
        if (vertIN > 5.25){
            clawOpen();
            Wrist.setPosition(.5);
            drivePos();
            state = "Score Specimen Release";
        } else {
            VertTarget = 5.75;
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
        telemetry.addData("WristTarget", wristTarget);
        telemetry.addData("MiniArm Error", miniArmPID.getPositionError());
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

//        if (digitalTouch.isPressed()) {
//            vertOffest += vertIN; //TODO fix this shit... always sets the thing to zero prolly cus i set it as a digiital device and not a rev touch sensor but to dumb to fix
//        }


        vertTicks = (VertLeft.getCurrentPosition() + -VertRight.getCurrentPosition())/2.0;
        vertIN = vertTicks / 450 - vertOffest;

        double output = vertExtension.calculate(vertIN, VertTarget);
        extendArm(output);
    }

    public void setMiniArmAngle() {


        double currentPosition = (MiniArm.getCurrentPosition() / TICKS_PER_DEGREE) - vertOffest;
        double pidOutput = miniArmPID.calculate(currentPosition, targetAngleDegreesMiniArm);

        MiniArm.setPower(-pidOutput);
    }
}