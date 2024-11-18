package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmSubsystem {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1, gamepad2;

    DcMotor armMotorLeft;
    DcMotor armMotorRight;
    DcMotor extenderLeft, extenderRight; //TODO Check 560 ticks per rev
    //TODO Limit C=42/cos(x)

    InterpLUT IntP = new InterpLUT(), IntI = new InterpLUT(), IntD = new InterpLUT(), IntF = new InterpLUT();
    PIDFController Pid;
    public static PIDController ExtenderPid;

    CRServo intakeServo;

    //Set in dashboard
    public static boolean debugPID = false;
    public static double[] pid1 = new double[]{0,0,0,0};
    public static double[] pid2 = new double[]{0,0,0,0};
    public static double[] pide = new double[]{0.08,0,0};
    public static double ff;
    public static double power_mult;
    public static double target;
    double TICKS_PER_DEGREE = 360.0/8192.0;
    public GamepadEx gamepad1ex;

    /*TODO*/public static double EXTENDER_TICKS = (10.0 / 257.5)/3;//4.5/26; // We want inches per tick. So we will use the motors to extend the arm out 10 inches and then do: 10 / however many ticks we get.
    double extenderPositionTicks = 0;
    public static PIDController extenderPID = new PIDController(0,0,0);
    double extenderInches = 0;
    double extenderTicks = 0;
    double extensionMax = 42;
    double extenderTarget = 16;

    public ArmSubsystem(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
        gamepad1ex = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotorRight = hardwareMap.get(DcMotor.class, "BigArmRight");
        armMotorLeft = hardwareMap.get(DcMotor.class, "BigArmLeft");

        extenderLeft = hardwareMap.get(DcMotor.class, "ExtenderLeft");
        extenderRight = hardwareMap.get(DcMotor.class, "ExtenderRight");

        //TODO - Not sure which one needs to be reversed.
        extenderLeft.setDirection(DcMotor.Direction.REVERSE);
        //extenderRight.setDirection(DcMotorSimple.Direction.REVERSE);

        power_mult = 0.85;

        pid1 = new double[]{0.02, 0, 0, 0.0009};
        pid2 = new double[]{0.02, 0, 0, 0.0009};

        //pid1 = new double[]{0.036, 0, 0, 0.0009};
        //pid2 = new double[]{0.036, 0, 0, 0.0009};

        extenderPID = new PIDController(0,0,0);

//        IntP.add(-55, pid1[0]);
//        IntP.add(365, pid1[0]);
//        IntI.add(-55,pid1[1]);
//        IntI.add(365, pid1[1]);
//        IntD.add(-55, pid1[2]);
//        IntD.add(365, pid1[2]);
//        IntF.add(-55,pid1[3]);
//        IntF.add(365, pid1[3]);

        // Cool Pid Interpolation
        IntP.add(-33, pid1[0]);
        IntP.add(180, pid2[0]);
        //IntP.add(120, 1);
        IntI.add(-33,pid1[1]);
        IntI.add(180, pid2[1]);
        //IntI.add(120, 1);
        IntD.add(-33, pid1[2]);
        IntD.add(180, pid2[2]);
        //IntD.add(120, 1);
        IntF.add(-33,pid1[3]);
        IntF.add(180, pid2[3]);
        //IntF.add(120, 1);

        IntP.createLUT(); IntI.createLUT(); IntD.createLUT(); IntF.createLUT();

        ff = 0.1; target = 0;

        Pid = new PIDFController(0,0,0,0);
        Pid.setPIDF(
                IntP.get(0),
                IntI.get(0),
                IntD.get(0),
                IntF.get(0));

        intakeServo = hardwareMap.get(CRServo.class, "IS");
    }

    public void loop() {
        FtcDashboard.getInstance().updateConfig();

        extenderPID.setPID(pide[0], pide[1], pide[2]);

        double armPos = -armMotorRight.getCurrentPosition();
        double armDeg = ((-armMotorRight.getCurrentPosition() * TICKS_PER_DEGREE) % 360);

        //Get the average ticks - Should help balance off inaccuracy.
        extenderPositionTicks = -(extenderLeft.getCurrentPosition() - extenderRight.getCurrentPosition()) / 2.0;
        extenderInches = 16 + extenderPositionTicks * EXTENDER_TICKS;
        extensionMax = 33;//42-21 * Math.cos(armDeg);

        telemetry.addData("-33", armDeg >= -33);
        telemetry.addData("70", armDeg <= 70);
        telemetry.addData("< 100", armDeg < 100);
        telemetry.addData("> 100", armDeg > 100);

        if ((armDeg > -33) && ((armDeg < 65))){
            extensionMax = 33;
            if(extenderInches > 33){
                extenderTarget = 33;
            }
        } else if ((armDeg > 65) && (armDeg < 110)){
            extensionMax = 56;
        } else if (armDeg > 110) {
            extensionMax = 20;
            if (extenderTarget > 20){
                extenderTarget = 20;
            }
        }

        if(gamepad2.a){
            intakeServo.setPower(1);
        } else if (gamepad2.y) {
            intakeServo.setPower(-1);
        } else {
            intakeServo.setPower(0);
            //intakeServo.disable();
        }

        extender(-extenderPID.calculate(extenderInches, extenderTarget));

//        if(gamepad2.dpad_up){
//            extender(0.5);
//        } else if (gamepad2.dpad_down && (extenderInches < extensionMax)) {
//            extender(-1);
//        } else {
//            extender(0);
//        }

        if(gamepad2.b){
            target = 75;
        }
        if(gamepad2.x){
            extenderTarget = 52;
        }

        if(gamepad2.dpad_right && target >= 118){
            target = target - 1;
        } else if(gamepad2.dpad_left){
            target = target - 1.8;
        } if(gamepad2.dpad_right){
            target = target + 1.8;
        }

        if(extenderTarget >= extensionMax) {
            extenderTarget = extenderTarget - 1.5;
        }else if (extenderTarget <= 14){
            extenderTarget = extenderTarget + 0.1;
        }else if(gamepad2.right_bumper){
            extenderTarget = extenderTarget + 1;
        } else if (gamepad2.left_bumper) {
            extenderTarget = extenderTarget - 1;
        }

//        if(gamepad2.a){
//            extender(ExtenderPid.calculate(encoderInches, encoderTarget));
//            if(gamepad2.dpad_up){
//                extensionLength(1);
//            } else if (gamepad2.dpad_down) {
//                extensionLength(-1);
//            }} else {
//            extender(0);
//        }

        //TODO Test
        Pid.setPIDF(
                IntP.get(armDeg),
                IntI.get(armDeg),
                IntD.get(armDeg),
                IntF.get(armDeg));

        double output = -Pid.calculate(armDeg, target);
        arm(output);

        telemetry.addData("ARMAVERAGEPOWER", (armMotorLeft.getPower() + armMotorRight.getPower()/2));

        FtcDashboard.getInstance().updateConfig();

        telemetry.addData("Encoder Pos:", armPos);
        telemetry.addData("Setpoint:", target);
        telemetry.addData("Deg", armDeg);
        telemetry.addData("Output:", Pid.calculate(armPos,target));

        telemetry.addData("P", Pid.getP());

        telemetry.addData("Measured Inches", extenderInches);
        telemetry.addData("ExtenderTarget", extenderTarget);
        telemetry.addData("EncoderTicks", extenderPositionTicks);
        telemetry.addData("Max Extension", extensionMax);

        telemetry.addData("IS Power:", intakeServo.getPower());

        if(debugPID){
            telemetry.addData("IntP:", IntP.get(armDeg));
            telemetry.addData("IntI:", IntI.get(armDeg));
            telemetry.addData("IntD:", IntD.get(armDeg));
            telemetry.addData("IntF:", IntF.get(armDeg));

            telemetry.addData("P:", Pid.getP());
            telemetry.addData("I:", Pid.getI());
            telemetry.addData("D:", Pid.getD());
            telemetry.addData("F:", Pid.getF());
        }
    }

    public void extender(double power){
        extenderLeft.setPower(power);
        extenderRight.setPower(-power);
    }

    public void arm(double power){
        armMotorLeft.setPower(-power);
        armMotorRight.setPower(power);
    }

    public double extensionLength(double control){
        if (control > 0 && (extenderInches >= extensionMax - 1)){
            return extenderTicks;
        } else if (control > 0 && !(extenderInches >= extensionMax - 1)) {
            extenderTicks += 0.1;
        } else if (control < 0 && (extenderInches <= -2)) {
            extenderTicks -= 0.1;
        }
        return extenderTicks;
    }
}
