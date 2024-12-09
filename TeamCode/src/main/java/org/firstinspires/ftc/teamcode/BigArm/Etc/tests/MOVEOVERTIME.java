package org.firstinspires.ftc.teamcode.Etc.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.pedroPathing.util.PIDFController;

public class MOVEOVERTIME {
    private final PIDFController armPidController;
    private final PIDController extenderPidController;
    private final DcMotor armMotorLeft;
    private final DcMotor armMotorRight;
    private final DcMotor extenderLeft;
    private final DcMotor extenderRight;

    private final ElapsedTime timer;

    // Motion profile parameters
    private double startArmAngle;
    private double targetArmAngle;
    private double startExtenderPosition;
    private double targetExtenderPosition;
    private double totalMovementTime;

    double TICKS_PER_DEGREE = 360.0/8192.0;

    public MOVEOVERTIME(PIDFController armPidController, PIDController extenderPidController, DcMotor armMotorLeft, DcMotor armMotorRight, DcMotor extenderLeft, DcMotor extenderRight, ElapsedTime timer) {
        this.armPidController = armPidController;
        this.extenderPidController = extenderPidController;
        this.armMotorLeft = armMotorLeft;
        this.armMotorRight = armMotorRight;
        this.extenderLeft = extenderLeft;
        this.extenderRight = extenderRight;
        this.timer = timer;
    }

    public void executeTimedMotion(
            double targetArm,
            double targetExtension,
            double movementTime
    ) {
        // Capture current positions
        startArmAngle = getCurrentArmAngle();
        startExtenderPosition = getCurrentExtenderPosition();

        // Set target positions
        targetArmAngle = targetArm;
        targetExtenderPosition = targetExtension;
        totalMovementTime = movementTime;

        // Reset timer
        timer.reset();

        // Execute motion profile
        while (timer.seconds() < totalMovementTime) {
            // Calculate progress (0 to 1)
            double progress = timer.seconds() / totalMovementTime;

            // Apply smooth interpolation (cubic easing)
            double smoothProgress = cubicEaseInOut(progress);

            // Interpolate current positions
            double currentArmAngle = interpolate(
                    startArmAngle,
                    targetArmAngle,
                    smoothProgress
            );

            double currentExtenderPosition = interpolate(
                    startExtenderPosition,
                    targetExtenderPosition,
                    smoothProgress
            );

            // Calculate PID outputs
            double armOutput = armPidController.calculate(currentArmAngle);

            double extenderOutput = extenderPidController.calculate(
                    getCurrentExtenderPosition(),
                    currentExtenderPosition
            );

            // Apply motor powers
            setArmPower(armOutput);
            setExtenderPower(extenderOutput);

            // Optional: Add small delay to prevent overwhelming the control loop
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }

        // Ensure final position
        setArmTarget(targetArmAngle);
        setExtenderTarget(targetExtenderPosition);
    }

    // Cubic easing function for smooth acceleration/deceleration
    private double cubicEaseInOut(double t) {
        return t < 0.5
                ? 4 * t * t * t
                : 1 - Math.pow(-2 * t + 2, 3) / 2;
    }

    // Linear interpolation between start and end values
    private double interpolate(
            double start,
            double end,
            double progress
    ) {
        return start + (end - start) * progress;
    }

    // Helper methods (implementations would match previous refactored code)
    private double getCurrentArmAngle() {
        return -armMotorRight.getCurrentPosition() * TICKS_PER_DEGREE % 360;
    }

    private double getCurrentExtenderPosition() {
        double avgTicks = -(extenderLeft.getCurrentPosition() - extenderRight.getCurrentPosition()) / 2.0;
        return 16 + avgTicks * EXTENDER_TICKS;
    }

    private void setArmPower(double power) {
        armMotorLeft.setPower(-power);
        armMotorRight.setPower(power);
    }

    private void setExtenderPower(double power) {
        extenderLeft.setPower(power);
        extenderRight.setPower(-power);
    }

    // Example usage method
    public void exampleUsage() {
        // Move arm to 45 degrees and extend to 24 inches over 2 seconds
        executeTimedMotion(
                45.0,   // Target arm angle
                24.0,   // Target extension (inches)
                2.0     // Movement time (seconds)
        );
    }
}