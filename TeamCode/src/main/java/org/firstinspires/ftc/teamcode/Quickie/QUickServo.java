package org.firstinspires.ftc.teamcode.Quickie;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "Quickie")
public class QUickServo extends OpMode {

    Servo armServoL;
    Servo armServoR;
    public static double pos = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armServoL = hardwareMap.get(Servo.class, "ArmServoL");
        armServoR = hardwareMap.get(Servo.class, "ArmServoR");
        armServoL.resetDeviceConfigurationForOpMode();
        armServoR.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void loop() {
        armServoR.setPosition(pos);
        armServoL.setPosition(pos);
        //armServoR.setPosition(pos);

        telemetry.addData("Pos", pos);
        telemetry.addData("Servo L", armServoL.getPosition());
        telemetry.addData("Servo R", armServoR.getPosition());
        if(gamepad1.a){
            pos = pos + 0.2;
        } else if (gamepad1.b) {
            pos = pos - 0.2;
        }
        if(gamepad1.x){
            armServoL.setPosition(0.05);
        }
        if(gamepad1.y){
            armServoL.setPosition(-0.05);
        }
    }
}
