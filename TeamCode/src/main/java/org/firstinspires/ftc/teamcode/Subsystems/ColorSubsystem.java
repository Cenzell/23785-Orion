package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ColorSubsystem {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    public ColorSubsystem(){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void init(){

    }

    public void loop(){

    }
}
