package frc.team2225.robot.command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2225.robot.Robot;
import frc.team2225.robot.Vector2D;

public class BallPlace extends Command{
    TalonSRX left;
    TalonSRX right;
    boolean curveLeft = false;
    boolean curveRight = false;

    public BallPlace(TalonSRX left, TalonSRX right, boolean curveLeft, boolean curveRight) {
        this.left = left;
        this.right = right;
        this.curveLeft = curveLeft;
        this.curveRight = curveRight;
    }
    public void grab(boolean on) {
        right.set(ControlMode.PercentOutput, on ? 1 : 0);
        left.set(ControlMode.PercentOutput, on ? 1 : 0);
    }
    public void push(boolean on) {
        right.set(ControlMode.PercentOutput, on ? -1 : 0);
        left.set(ControlMode.PercentOutput, on ? -1 : 0);
    }
    public void curvedPush(boolean on, boolean isLeft) {
        if (isLeft) {
            right.set(ControlMode.PercentOutput, on ? -0.5 : 0);
            left.set(ControlMode.PercentOutput, on ? -1 : 0);
        }
        else {
            right.set(ControlMode.PercentOutput, on ? -1 : 0);
            left.set(ControlMode.PercentOutput, on ? -0.5 : 0);
        }
    }
    @Override
    protected void initialize() {
        //TODO: Set elevator height to high, medium or low; parameters will need to be added for that

        Robot.drivetrain.translate(new Vector2D(0, 10));

        if (curveLeft) {
            this.curvedPush(true, true);
        }
        else if (curveRight) {
            this.curvedPush(true, false);
        }
        else {
            this.push(true);
        }


        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            DriverStation.reportWarning("The robot's clock is being interrupted", false);
        }

        this.push(false);
        Robot.drivetrain.translate(new Vector2D(0, -10));
        //TODO: Set elevator height back
    }
    @Override
    protected void execute() {
    }
    @Override
    protected boolean isFinished() {
        return false;
    }
}


