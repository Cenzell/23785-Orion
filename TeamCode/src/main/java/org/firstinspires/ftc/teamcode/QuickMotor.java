package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@TeleOp(name="QuickArm")
public class QuickMotor extends OpMode {

    DcMotor armMotorLeft;
    @Override
    public void init() {
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "BigArm");
    }

    @Override
    public void loop() {

        if (gamepad2.a) {
            armMotorLeft.setPower(1);
        } else if (gamepad2.b) {
            armMotorLeft.setPower(0);
        } else {
            armMotorLeft.setPower(0);
        }
    }
}
