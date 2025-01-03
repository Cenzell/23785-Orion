package org.firstinspires.ftc.lib.AutoLib;

import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public abstract class AutoLib extends OpMode {
    
    // Core timing and state management
    protected ElapsedTime runtime = new ElapsedTime();
    protected ElapsedTime stageTimer = new ElapsedTime();

    // Autonomous routine builder
    protected AutoRoutineBuilder routineBuilder;

    @Override
    public void init() {
        // Initialize hardware and build routine
        initHardware();
        routineBuilder = createRoutine();
        runtime.reset();
    }

    @Override
    public void loop() {
        // Global timeout check
        if (runtime.seconds() >= 30) {
            requestOpModeStop();
            return;
        }

        // Execute current stage of routine
        routineBuilder.update();

        // Telemetry for debugging
        telemetry.addData("Current Stage", routineBuilder.getCurrentStageName());
        telemetry.addData("Runtime", runtime.seconds());
        telemetry.update();
    }

    // Abstract method to initialize robot-specific hardware
    protected abstract void initHardware();

    // Abstract method to define the specific autonomous routine
    protected abstract AutoRoutineBuilder createRoutine();

    // Utility class for building autonomous routines
    public static class AutoRoutineBuilder {
        private List<AutoStage> stages = new ArrayList<>();
        private int currentStageIndex = 0;
        private ElapsedTime stageTimer = new ElapsedTime();

        // Add a stage to the routine
        public AutoRoutineBuilder addStage(AutoStage stage) {
            stages.add(stage);
            return this;
        }

        // Update method to progress through stages
        public void update() {
            if (currentStageIndex >= stages.size()) return;

            AutoStage currentStage = stages.get(currentStageIndex);

            // Execute current stage
            if(!currentStage.isFailed()){
                currentStage.run();
            } else {
                currentStage.fail();
            }

            // Check for stage completion
            if (currentStage.isComplete()) {
                currentStageIndex++;
                if (currentStageIndex < stages.size()) {
                    // Reset timer for next stage
                    stageTimer.reset();
                }
            }
        }

        // Get name of current stage for telemetry
        public String getCurrentStageName() {
            if (currentStageIndex < stages.size()) {
                return stages.get(currentStageIndex).getName();
            }
            return "Completed";
        }
    }

    // Interface for autonomous stages
    public interface AutoStage {
        // Run the current stage
        void run();

        void fail();

        // Check if stage is complete
        boolean isComplete();

        // Check if the stage failed
        boolean isFailed();

        // Get stage name for logging
        String getName();
    }

    public static class GenericState implements AutoStage {
        private String name;
        private Runnable action;
        private Runnable failAction;
        private Supplier<Boolean> completionCheck;
        private double timeout;
        private ElapsedTime timer = new ElapsedTime();

        public GenericState(String name, Runnable moveAction,
                         Supplier<Boolean> completionCheck,
                         double timeout) {
            this.name = name;
            this.action = moveAction;
            this.completionCheck = completionCheck;
            this.timeout = timeout;
        }

        @Override
        public void run() {
            action.run();
        }

        @Override
        public void fail() {
            failAction.run();
        }

        @Override
        public boolean isComplete() {
            return false;
        }

        @Override
        public boolean isFailed() {
            return false;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    // Example implementation of a move stage
    public static class MoveStage implements AutoStage {
        private String name;
        private Runnable moveAction;
        private Runnable failAction;
        private Supplier<Boolean> completionCheck;
        private double timeout;
        private ElapsedTime timer = new ElapsedTime();
        private Path path;
        private Follower follower;

        public MoveStage(String name, Runnable moveAction,
                         Supplier<Boolean> completionCheck,
                         double timeout) {
            this.name = name;
            this.moveAction = moveAction;
            this.completionCheck = completionCheck;
            this.timeout = timeout;
        }

        public MoveStage(String name, Path path, Follower follower){
            this.name = name;
            this.path = path;
            this.follower = follower;
        }

        @Override
        public void run() {
            moveAction.run();
            if(moveAction != null){
                moveAction.run();
            } else {
                follower.followPath(path);
            }
        }

        @Override
        public void fail() {
            failAction.run();
        }

        @Override
        public boolean isComplete() {
            return completionCheck.get() || timer.seconds() >= timeout;
        }

        @Override
        public boolean isFailed() {
            return false;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    // Example implementation of a scoring stage
    public static class ScoreStage implements AutoStage {
        private String name;
        private Runnable failAction;
        private Runnable scoreAction;
        private double duration;
        private ElapsedTime timer = new ElapsedTime();

        public ScoreStage(String name, Runnable scoreAction, double duration) {
            this.name = name;
            this.scoreAction = scoreAction;
            this.duration = duration;
        }

        @Override
        public void run() {
            scoreAction.run();
        }

        @Override
        public void fail() {
            failAction.run();
        }

        @Override
        public boolean isComplete() {
            return timer.seconds() >= duration;
        }

        @Override
        public boolean isFailed() {
            return false;
        }

        @Override
        public String getName() {
            return name;
        }
    }
}

