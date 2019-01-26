package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2225.robot.Robot;
import frc.team2225.robot.Vector2D;

public class MoveForward extends Command {
    public MoveForward() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        Robot.drivetrain.translate(new Vector2D(0, 10));
    }

    @Override
    protected void execute() {
        super.execute();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
