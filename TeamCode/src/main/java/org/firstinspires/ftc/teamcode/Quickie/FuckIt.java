package org.firstinspires.ftc.teamcode.Quickie;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FuckIt", group="FUCKASS")
public class FuckIt extends OpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBack = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBack = hardwareMap.get(DcMotor.class, "BackRight");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            leftFront.setPower(0.2);
            rightFront.setPower(0.2);
        } else if (gamepad1.b) {
            leftBack.setPower(0.2);
            rightBack.setPower(0.2);
        }
    }
}
