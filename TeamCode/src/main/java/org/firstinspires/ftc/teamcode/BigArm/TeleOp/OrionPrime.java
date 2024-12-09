package org.firstinspires.ftc.teamcode.BigArm.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BigArm.Subsystems.ArmNeo;
import org.firstinspires.ftc.teamcode.BigArm.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.BigArm.Subsystems.IntakeSubsystem;

@TeleOp(name = "OrionPrime", group = "Orion")
public class OrionPrime extends OpMode {

    ArmNeo armSubsystem;
    IntakeSubsystem intakeSubsystem;
    DriveSubsystem driveSubsystem;

    enum ArmState {
        INTAKE_CLOSE,
        INTAKE_FAR,
        SCORE_LOW,
        SCORE_HIGH,
        RESET,
        TRAVEL,
        MANUAL
    }

    boolean a = false, x = false, y = false, b = false, ps = false, option = false, lStick = false, rStick = false, trackpad = false;

    ArmState state = ArmState.RESET;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armSubsystem = new ArmNeo(telemetry, hardwareMap, gamepad1, gamepad2); //Fine
        armSubsystem.init();

        intakeSubsystem = new IntakeSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        intakeSubsystem.init();

        driveSubsystem = new DriveSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        driveSubsystem.init();
    }

    @Override
    public void loop() {
        armSubsystem.loop();
        intakeSubsystem.loop();
        driveSubsystem.loop();

        /// INTAKE ///
        if (gamepad2.a) {
            intakeSubsystem.setPower(1);
        } else if (gamepad2.y) {
            intakeSubsystem.setPower(-1);
        } else {
            intakeSubsystem.setPower(0);
        }

        /// State Cycle ///
        if (gamepad2.b) {
            armTarget(75);
        }

        /// ARM ANGLE ///
        if (gamepad2.dpad_right){
            armSubsystem.lowerArm();
        } else if (gamepad2.dpad_left) {
            armSubsystem.raiseArm();
        }

        //Toggle between High and Low bucket.
        if(gamepad2.x & !x){
            if(state == ArmState.SCORE_HIGH){
                state = ArmState.SCORE_LOW;
            } else {
                state = ArmState.SCORE_HIGH;
            }
        } else if (!gamepad2.x) {
            x = false;
        }

        //Toggle between far and close intake.
        if(gamepad2.b && !b){
            if(state == ArmState.INTAKE_FAR){
                state = ArmState.INTAKE_CLOSE;
            } else {
                state = ArmState.INTAKE_FAR;
            }
        } else if (!gamepad2.b) {
            b = false;
        }

        if(gamepad2.a){
            state = ArmState.TRAVEL;
        }

        if (state == ArmState.RESET) {
            armTarget(3);
            extension(16);
        } else if (state == ArmState.MANUAL) {
            //do nothing
        } else if (state == ArmState.INTAKE_CLOSE) {
            armTarget(10);
            extension(18);
        } else if (state == ArmState.INTAKE_FAR) {
            armTarget(0);
            extension(16);
        } else if (state == ArmState.SCORE_LOW) {
            armTarget(0);
            extension(16);
        } else if (state == ArmState.SCORE_HIGH) {
            armTarget(75);
            extension(16);
        } else if (state == ArmState.TRAVEL) {
            armTarget(45);
            extension(25);
        } else {telemetry.addLine("!!! Bad State !!!");}

        telemetry.addData("State:", state.name());
    }

    void armInt(double armPos, double ext, double time){

    }

    void armTarget(double target){
        armSubsystem.armTarget(target);
    }
    void extension(double extension){
        armSubsystem.extension(extension);
    }
}
