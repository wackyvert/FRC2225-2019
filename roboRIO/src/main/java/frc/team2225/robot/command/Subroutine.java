package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.command.Command;

import java.util.List;

/**
 * A state machine that can be run in periodically in an iterative environment
 * <p>
 * Consists of Stages, which are initialized, run until they are done, and then finalized in order
 * <p>
 * To create a subroutine, override the getStages method to return a list of stages
 */
public abstract class Subroutine extends Command {
    private Stage[] stages;
    private boolean isFinished = false;
    private int stageIndex = 0;

    @Override
    protected void initialize() {
        this.stages = (Stage[]) getStages().toArray();
        stages[0].init();
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

    /**
     * A stage in a Subroutine
     */
    interface Stage {

        default void init() {
        }

        default void execute() {
        }

        default void finish() {
        }

        boolean isDone();
    }

    /**
     * Creates a stage that finishes after a set duration
     */
    abstract class TimedStage implements Stage {
        long startTime;
        long duration;

        public TimedStage(long durationMs) {
            startTime = System.currentTimeMillis();
            duration = durationMs;
        }

        protected boolean isDurationExpired() {
            return System.currentTimeMillis() > startTime + duration;
        }

        @Override
        public boolean isDone() {
            return isDurationExpired();
        }
    }
}
