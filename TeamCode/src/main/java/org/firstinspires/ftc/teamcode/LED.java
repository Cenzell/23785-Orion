package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.AnalogCommands;

import java.util.jar.Attributes;

@TeleOp(name = "LED")
public class LED extends OpMode {

    Servo led;
    Timing.Timer LedTimer;
    DigitalChannel LED;


    @Override
    public void init() {

        //led = hardwareMap.get(DigitalChannel.class, "light");

        //LedTimer = new Timing.Timer(200);

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            //led.setPosition(1);
        } else {
            //led.setPosition(0);
        }
        //telemetry.addData("Power?", led.getPosition());
    }
}
