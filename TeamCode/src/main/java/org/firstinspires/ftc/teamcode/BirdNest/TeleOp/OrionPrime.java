package org.firstinspires.ftc.teamcode.BirdNest.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BigArm.Subsystems.ArmNeo;
import org.firstinspires.ftc.teamcode.BigArm.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.BigArm.Subsystems.IntakeSubsystem;

@TeleOp(name = "BirdNest", group = "Orion")
public class OrionPrime extends OpMode {
    DriveSubsystem driveSubsystem;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveSubsystem = new DriveSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        driveSubsystem.init();
    }

    @Override
    public void loop() {
        driveSubsystem.loop();

    }

}
