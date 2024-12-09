package org.firstinspires.ftc.teamcode.BigArm.Subsystems;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem{

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public Gamepad gamepad1, gamepad2;
    public CRServo intakeServo;
    public RevColorSensorV3 colorSensor;
    public NormalizedRGBA colors;
    double[] rgb = new double[] {0,0,0};
    public LED redLed;
    public LED greenLed;
    ElapsedTime ledTime;

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
        ledTime = new ElapsedTime();
        intakeServo = hardwareMap.get(CRServo.class, "IS");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        redLed = hardwareMap.get(LED.class, "redLed");
        greenLed = hardwareMap.get(LED.class, "greenLed");
        colors = colorSensor.getNormalizedColors();
        updateSample();
        ledTime.reset();
    }

    public void loop() {
        colors = colorSensor.getNormalizedColors();
        updateSample();

        rgb = new double[]{colors.red, colors.green, colors.blue};

        telemetry.addData("SAMPLE:", sample.name());
        telemetry.addData("POSSESSION:", possession.name());

        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("RLed", redLed.isLightOn());
        telemetry.addData("GLed", greenLed.isLightOn());
        telemetry.addData("Distance:", colorSensor.getDistance(DistanceUnit.INCH));

        if(sample != Sample.NONE && ledTime.seconds() > 1.5){
            gamepad2.rumble(1);
            redLed.off();
            greenLed.on();
            ledTime.reset();
        } else if (sample == Sample.NONE && ledTime.seconds() > 1.5) {
            redLed.on();
            greenLed.off();
            ledTime.reset();
        } else {
            redLed.off();
            greenLed.off();
            telemetry.addLine("/// LED ERROR ///");
        }
        FtcDashboard.getInstance().updateConfig();

    }

    public void updateSample(){
        if(colorSensor.getDistance(DistanceUnit.INCH) < 1) {
            possession = Possession.HAS_PIECE;
        } else{
            possession = Possession.NO_PIECE;
            redLed.on();
        }

        if((colors.red > colors.blue) && (colors.red > colors.green)){
            sample = Sample.RED;
        } else if ((colors.blue > colors.red) && (colors.blue > colors.green)){
            sample = Sample.BLUE;
        } else if ((colors.red > colors.blue) && (colors.green > colors.blue)){
            sample = Sample.YELLOW;
        } else{
            sample = Sample.NONE;
        }
    }

    public void setPower(double power){
        intakeServo.setPower(power);
    }

    public Possession hasPiece(){
        return possession;
    }

}
