package org.firstinspires.ftc.teamcode.Subsystems;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class IntakeSubsystem{

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public Gamepad gamepad1, gamepad2;
    public CRServo intakeServo;
    public RevColorSensorV3 colorSensor;
    public NormalizedRGBA colors;
    double[] rgb = new double[] {0,0,0};
    public DigitalChannel redLed;
    public DigitalChannel greenLed;

    public enum Possession {
        HAS_PIECE,
        NO_PIECE
    }

    public enum Sample {
        BLUE,
        RED,
        YELLOW,
        NONE
    }

    public Possession possession = Possession.NO_PIECE;
    public Sample sample = Sample.NONE;

    public IntakeSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        intakeServo = hardwareMap.get(CRServo.class, "IS");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        redLed = hardwareMap.get(DigitalChannel.class, "redLed");
        greenLed = hardwareMap.get(DigitalChannel.class, "greenLed");
        colors = colorSensor.getNormalizedColors();
        updateSample();
    }

    public void loop() {
        colors = colorSensor.getNormalizedColors();
        updateSample();

        rgb = new double[]{colors.red, colors.green, colors.blue};

        telemetry.addData("SAMPLE:", sample.name());
        telemetry.addData("POSSESSION:", possession.name());

        telemetry.addData("RGB:", rgb);
        telemetry.addData("Distance:", colorSensor.getDistance(DistanceUnit.INCH));

        if(sample != Sample.NONE){
            gamepad2.rumble(1);
            redLed.setState(false);
            greenLed.setState(true);
        } else {
            redLed.setState(true);
            greenLed.setState(false);
        }
        FtcDashboard.getInstance().updateConfig();

    }

    public void updateSample(){
        if(colorSensor.getDistance(DistanceUnit.INCH) < 1) {
            possession = Possession.HAS_PIECE;
        } else{
            possession = Possession.NO_PIECE;
        }

        if(colors.red > 0.1 && (colors.blue < 0.1 || colors.green < 0.1)){
            sample = Sample.RED;
        } else if (colors.blue > 0.1 && (colors.red < 0.1 || colors.green < 0.1)){
            sample = Sample.BLUE;
        } else if (colors.blue < 0.1 && (colors.red > 0.1 || colors.green > 0.1)){
            sample = Sample.YELLOW;
        } else{
            sample = Sample.NONE;
        }
    }

    public void setPower(double power){
        intakeServo.setPower(power);
    }

}
