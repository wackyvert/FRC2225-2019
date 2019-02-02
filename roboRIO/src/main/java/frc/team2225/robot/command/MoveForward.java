package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2225.robot.Robot;
import frc.team2225.robot.Vector2D;
import frc.team2225.robot.subsystem.Drivetrain;

public class MoveForward extends Command {

    Drivetrain.CheckPosition pos;

    public MoveForward() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        pos = Robot.drivetrain.translate(new Vector2D(0, 10));
    }

    @Override
    protected void execute() {
        System.out.println(pos.isDone());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
