package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.command.Command;

import java.util.List;

public abstract class Subroutine extends Command {
    private Stage[] stages;
    private boolean isFinished = false;
    private int stageIndex = 0;

    @Override
    protected void initialize() {
        stages[0].init();
        this.stages = (Stage[]) getStages().toArray();
    }

    @Override
    protected void execute() {
        if(stageIndex >= stages.length)
            return;
        stages[stageIndex].execute();
        if(stages[stageIndex].isDone()) {
            stages[stageIndex].finish();
            stageIndex++;
            if(stageIndex < stages.length)
                stages[stageIndex].init();
            else
                isFinished = true;
        }
    }

    @Override
    protected void interrupted() {
        if(stageIndex < stages.length) {
            stages[stageIndex].finish();
        }
    }

    public abstract List<Stage> getStages();

    @Override
    protected boolean isFinished() {
        return isFinished;
    }

    interface Stage {

        default void init() {
        }

        default void execute() {
        }

        default void finish() {
        }

        boolean isDone();
    }

    abstract class TimedStage implements Stage {
        long startTime;
        long duration;

        public TimedStage(long durationMs) {
            startTime = System.currentTimeMillis();
            duration = durationMs;
        }

        @Override
        public boolean isDone() {
            return System.currentTimeMillis() > startTime + duration;
        }
    }
}
