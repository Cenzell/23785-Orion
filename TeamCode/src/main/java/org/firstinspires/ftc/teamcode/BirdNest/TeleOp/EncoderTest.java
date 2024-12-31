package org.firstinspires.ftc.teamcode.BirdNest.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends OpMode {

    DcMotor h0, h1, h2, h3;
    DcMotor e0, e1, e2, e3;

    @Override
    public void init() {
        h0 = hardwareMap.get(DcMotor.class, "h0");
        h1 = hardwareMap.get(DcMotor.class, "h1");
        h2 = hardwareMap.get(DcMotor.class, "h2");
        h3 = hardwareMap.get(DcMotor.class, "h3");
        e0 = hardwareMap.get(DcMotor.class, "e0");
        e1 = hardwareMap.get(DcMotor.class, "e1");
        e2 = hardwareMap.get(DcMotor.class, "e2");
        e3 = hardwareMap.get(DcMotor.class, "e3");
    }

    @Override
    public void loop() {
        telemetry.addData("h0", h0.getCurrentPosition());
        telemetry.addData("h1", h1.getCurrentPosition());
        telemetry.addData("h2", h2.getCurrentPosition());
        telemetry.addData("h3", h3.getCurrentPosition());
        telemetry.addData("e0", e0.getCurrentPosition());
        telemetry.addData("e1", e1.getCurrentPosition());
        telemetry.addData("e2", e2.getCurrentPosition());
        telemetry.addData("e3", e3.getCurrentPosition());
        h0.setPower(1);
    }
}
