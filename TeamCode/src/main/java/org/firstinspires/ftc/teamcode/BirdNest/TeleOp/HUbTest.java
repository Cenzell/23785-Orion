package org.firstinspires.ftc.teamcode.BirdNest.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HUbTest extends OpMode {

    DcMotor test;

    @Override
    public void init() {
        test = hardwareMap.get(DcMotor.class, "h0");
    }

    @Override
    public void loop() {
        telemetry.addData("H0", test.getCurrentPosition());
    }
}
