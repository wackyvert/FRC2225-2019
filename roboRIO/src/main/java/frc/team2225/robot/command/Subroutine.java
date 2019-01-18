package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.command.Command;

import java.util.function.Predicate;
import java.util.function.Supplier;

public class Subroutine extends Command {
    private Stage[] stages;
    private boolean isFinished = false;
    private int stageIndex = 0;

    @Override
    protected void initialize() {
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

    public Subroutine(Stage[] stages) {
        this.stages = stages;
    }

    @Override
    protected boolean isFinished() {
        return isFinished;
    }

    class Stage {
        Runnable init;
        Runnable execute;
        Runnable finish;
        Supplier<Boolean> isDone;

        public Stage(Runnable init, Runnable execute, Runnable finish, Supplier<Boolean> isDone) {
            this.init = init;
            this.execute = execute;
            this.finish = finish;
        }

        void init() {init.run();}
        void execute() {execute.run();}
        void finish() {finish.run();}

        boolean isDone() {return isDone.get();}
    }
}
