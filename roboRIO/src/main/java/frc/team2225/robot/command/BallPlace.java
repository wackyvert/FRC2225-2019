package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2225.robot.Robot;
import frc.team2225.robot.Vector2D;

public class BallPlace extends Command{
    boolean curveLeft;
    boolean curveRight;
    boolean isFinished = false;

    public BallPlace(boolean curveLeft, boolean curveRight) {
        requires(Robot.rollerIntake);
        requires(Robot.drivetrain);
        this.curveLeft = curveLeft;
        this.curveRight = curveRight;
    }

    @Override
    protected void initialize() {
        //TODO: Set elevator height to high, medium or low; parameters will need to be added for that

        Robot.drivetrain.translate(new Vector2D(0, 10));

        if (curveLeft) {
            Robot.rollerIntake.curvedPush(true, true);
        }
        else if (curveRight) {
            Robot.rollerIntake.curvedPush(true, false);
        }
        else {
            Robot.rollerIntake.push(true);
        }


        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            DriverStation.reportWarning("The robot's clock is being interrupted", false);
        }

        Robot.rollerIntake.push(false);
        Robot.drivetrain.translate(new Vector2D(0, -10));
        //TODO: Set elevator height back

        isFinished = true;
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return isFinished;
    }
}


