package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import static java.lang.Math.round;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotionSubsystem {

    public ServoImplEx HoriExtR, HoriExtL, MiniExt, Wrist, Claw, IntakeFlip;
    public DcMotorEx VertRight, MiniArm, Intake, VertLeft;

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1, gamepad2;

    PIDFController VertExtension;
    Double armDeg, ForwardLimit, VertLimit; //TODO see if all of this is needed.

    private double distanceTime = 2000;
    private long startTime;
    public int clawset = 0;


    public MotionSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }

    public void init(){
        HoriExtL = (ServoImplEx) hardwareMap.get(Servo.class, "HoriExtR");
        HoriExtR = (ServoImplEx) hardwareMap.get(Servo.class, "HoriExtL");
        MiniExt = (ServoImplEx) hardwareMap.get(Servo.class, "MiniExt");
        Wrist = (ServoImplEx) hardwareMap.get(Servo.class, "Wrist");
        Claw = (ServoImplEx) hardwareMap.get(Servo.class, "Claw");
        IntakeFlip = (ServoImplEx) hardwareMap.get(Servo.class, "IntakeFlip");

        //TODO Put into Config
        VertRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "VertRight");
        MiniArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "MiniArm");
        Intake = (DcMotorEx) hardwareMap.get(DcMotor.class, "Intake");
        VertLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "VertLeft");

        VertExtension = new PIDFController(0,0,0,0); //TODO Make not all zero
        if (gamepad1.dpad_down) {MiniExt.setPosition(.60);} // this is to make it not hot
    }

    public void loop(){
        //if (gamepad1.dpad_up) {
        //        HoriExtL.setPosition(1);
        //        HoriExtR.setPosition(1);
        //   }
        //    if (gamepad1.dpad_down) {
        //        HoriExtL.setPosition(0);
        //        HoriExtR.setPosition(0);
        //    }

            if (gamepad1.dpad_up) {MiniExt.setPosition(1);}
            if (gamepad1.dpad_down) {MiniExt.setPosition(.55);}

            //if (gamepad1.a) {
            //    IntakeFlip.setPosition(0.8);
            //}else{
            //    IntakeFlip.setPosition(0.1);
            //}
            if (gamepad1.b && clawset == 1){
                Claw.setPosition(0.02);
                clawset = 0;
            }else if(gamepad1.b && clawset == 1){
                Claw.setPosition(0.25);
                clawset = 1;
            }

        telemetry.addData("Right Pos", HoriExtR.getPosition());
        telemetry.addData("Left Pos", HoriExtL.getPosition());
        telemetry.addData("MiniArm", MiniArm.getCurrentPosition());

        if(gamepad2.right_trigger > .01){
            MiniArm.setPower(gamepad2.right_trigger*gamepad2.right_trigger);
        }else if(gamepad2.left_trigger > .01){
            MiniArm.setPower(-(gamepad2.left_trigger*gamepad2.right_trigger));
        }else{
            MiniArm.setPower(0);
        }

        if(gamepad1.dpad_left){
            Wrist.setPosition(1);
        }else if(gamepad1.dpad_right){
            Wrist.setPosition(0);
        }else{
            Wrist.setPosition(.5);
        }


        if(gamepad1.a){
            intakePrep();
        }
        if(gamepad1.right_bumper) {
            HoriExtL.setPosition(.0);
            HoriExtR.setPosition(.0);
            IntakeFlip.setPosition(0);
            Intake.setPower(0);
        }else if(gamepad1.right_trigger >.05){
            HoriExtL.setPosition(.81);
            HoriExtR.setPosition(.81);
        }

        if(gamepad1.x){
            Intake.setPower(-1);
        }

        if(gamepad1.y){
            transfer();
            Intake.setPower(0);
        }


    }

    public void intakePrep(){
        IntakeFlip.setPosition(.8);
        Intake.setPower(1);
        telemetry.addData("State", "intakePrep");

    }

    public void drivePos(){
        HoriExtL.setPosition(0);
        HoriExtR.setPosition(0);
        IntakeFlip.setPosition(.25);
        Intake.setPower(0);
        //set wrist to position - cont servo idk how
        telemetry.addData("State", "drivePos");

    }

    public void transfer(){
        IntakeFlip.setPosition(.25);
        MiniExt.setPosition(.01);
        //set wrist to position - cont servo idk how
        telemetry.addData("State", "transfering");
    }

    public void wallPickup() {
        MiniExt.setPosition(1);
        //set wrist to position - cont servo idk how
        //set mini arm angle
        telemetry.addData("State", "wallPickup");
    }

    public void sampleScore(){
        telemetry.addData("State", "scoreSample");
    }

    public void specimenPrep(){
        telemetry.addData("State", "SpecimenPrep");
    }

    public void specimenScore(){
        telemetry.addData("State", "scoreSpecimen");
    }

    public void Limits(){
        //TODO: Find good limits and enable
        if(armDeg > 999){
            armDeg = 45.0;
        } else if (armDeg < -100){
            armDeg = 45.0;
        }
    }

    public void ExtendIntake(){
        HoriExtR.setPosition(1);
        HoriExtL.setPosition(1);
    }

    public void RetractIntake(){
        HoriExtR.setPosition(0);
        HoriExtL.setPosition(0);
    }

    public void handOff(){
        HoriExtR.setPosition(0);
        HoriExtL.setPosition(0);
    }

}
