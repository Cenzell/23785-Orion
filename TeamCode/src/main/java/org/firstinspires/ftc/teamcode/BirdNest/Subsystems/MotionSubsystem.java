package org.firstinspires.ftc.teamcode.BirdNest.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionSubsystem {

    public ServoImplEx HoriExtR, HoriExtL, MiniExt, Wrist, Claw, IntakeFlip;
    public DcMotor VertRight, MiniArm, Intake, VertLeft;

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1, gamepad2;

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

        //VertRight = hardwareMap.get(DcMotor.class, "VertRight");
        //MiniArm = hardwareMap.get(DcMotor.class, "MiniArm");
        //Intake = hardwareMap.get(DcMotor.class, "Intake");
        //VertLeft = hardwareMap.get(DcMotor.class, "VertLeft");


    }

    public void loop(){
        if (gamepad1.dpad_up) {HoriExtR.setPosition(1);}
        if (gamepad1.dpad_down) {HoriExtR.setPosition(0);}

        telemetry.addData("Right Pos", HoriExtR.getPosition());
        telemetry.addData("Left Pos", HoriExtL.getPosition());


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
