package org.firstinspires.ftc.teamcode.BigArm.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Template {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1, gamepad2;

    public Template(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;


    }

    public void init() {

    }

    public void loop() {
        FtcDashboard.getInstance().updateConfig();



        FtcDashboard.getInstance().updateConfig();
    }
}
