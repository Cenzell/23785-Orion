package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

//@TeleOp(name="AS_ArmSubsystemTest")
public class AS_ArmSubsystemTest extends OpMode {

    ArmSubsystem armSubsystem;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armSubsystem = new ArmSubsystem(telemetry, hardwareMap, gamepad1, gamepad2); //Fine
        armSubsystem.init();
    }

    @Override
    public void loop() {
        armSubsystem.loop();
    }
}
