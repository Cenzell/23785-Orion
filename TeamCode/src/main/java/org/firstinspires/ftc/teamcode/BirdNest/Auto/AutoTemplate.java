package org.firstinspires.ftc.teamcode.BirdNest.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BirdNest.lib.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.BirdNest.lib.pedroPathing.constants.LConstants;

import org.firstinspires.ftc.teamcode.BirdNest.Subsystems.MotionSubsystem;

@Autonomous(name = "") //TODO: Add
public class AutoTemplate extends OpMode {
    Follower follower;
    MotionSubsystem motionSubsystem;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setPose(new Pose(0,0)); //TODO: Start pose here

        motionSubsystem.init();
    }

    @Override
    public void loop() {
        motionSubsystem.loop();

    }
}
