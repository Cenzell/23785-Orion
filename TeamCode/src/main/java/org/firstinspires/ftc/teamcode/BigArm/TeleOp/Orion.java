package org.firstinspires.ftc.teamcode.BigArm.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BigArm.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.BigArm.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.BigArm.Subsystems.IntakeSubsystem;

@Config
@TeleOp(name = "OrionA")
public class Orion extends OpMode {

    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    DriveSubsystem driveSubsystem;

    //public Telemetry telemetry;
    //public HardwareMap hardwareMap;
    //public Gamepad gamepad1, gamepad2;
    public CRServo intakeServo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armSubsystem = new ArmSubsystem(telemetry, hardwareMap, gamepad1, gamepad2); //Fine
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
    }
}
