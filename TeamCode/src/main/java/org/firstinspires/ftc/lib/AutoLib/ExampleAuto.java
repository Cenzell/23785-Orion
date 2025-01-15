package org.firstinspires.ftc.lib.AutoLib;

import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.lib.AutoLib.AutoLib.*;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

// Example Usage Demonstration
@Autonomous(name = "Simple Auto Example")
public class ExampleAuto extends OpMode {
    private Follower follower;
    private CRServo intakeServo;
    protected void initHardware() {
        // Initialize robot-specific hardware
    }
    
    protected AutoRoutineBuilder createRoutine() {
        return new AutoLib.AutoRoutineBuilder()
                .addStage(new MoveStage(
                        "Move to Scoring Position",
                        () -> {
                            // Move to scoring position logic
                            PathBuilder path = new PathBuilder();
                                path.addPath(new BezierLine(
                                            new Point(60.000, 30.000, Point.CARTESIAN),
                                            new Point(60.000, 25.000, Point.CARTESIAN)
                                    ))
                                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                    .build();
                            follower.followPath(path.build());
                        },
                        () -> !follower.isBusy(),
                        3.0  // 3-second timeout
                ))
                .addStage(new ScoreStage(
                        "Score Preload",
                        () -> {
                            // Scoring mechanism activation
                            intakeServo.setPower(-1);
                        },
                        1.5  // Hold scoring for 1.5 seconds
                ))
                .addStage(new MoveStage(
                        "Park",
                        () -> {
                            // Move to parking position
                            Path parkingPath = null;
                            follower.followPath(parkingPath);
                        },
                        () -> !follower.isBusy(),
                        3.0  // 3-second timeout
                ));
    }

    @Override
    public void init() {
        Pose startPose = new Pose(0,0);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        createRoutine().update();
        telemetry.addData("State", createRoutine().getCurrentStageName());
    }
}
