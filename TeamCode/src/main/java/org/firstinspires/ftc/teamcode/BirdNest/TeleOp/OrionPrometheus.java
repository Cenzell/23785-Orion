package org.firstinspires.ftc.teamcode.BirdNest.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.MotionSubsystem;

@TeleOp(name = "BirdNest", group = "Orion")
public class OrionPrometheus extends OpMode {
    DriveSubsystem driveSubsystem;
    MotionSubsystem motionSubsystem;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveSubsystem = new DriveSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        driveSubsystem.init();

        motionSubsystem = new MotionSubsystem(telemetry, hardwareMap, gamepad1, gamepad2);
        motionSubsystem.init();
    }

    @Override
    public void loop() {
        driveSubsystem.loop();

        motionSubsystem.loop();
    }

}
