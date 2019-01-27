package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2225.robot.Robot;
import frc.team2225.robot.Vector2D;

public class BallPlace extends Command{
    boolean curveLeft;
    boolean curveRight;
    boolean isFinished = false;
    int height;

    public BallPlace(boolean curveLeft, boolean curveRight, int height) {
        requires(Robot.rollerIntake);
        requires(Robot.drivetrain);
        this.curveLeft = curveLeft;
        this.curveRight = curveRight;
        this.height = height;
    }

    @Override
    protected void initialize() {
        //TODO: Set elevator height to high, medium or low; parameters will need to be added for that

        Robot.drivetrain.omniDrive(new Vector2D(0, 10), 0);

        if (height == 1) {
            //set to lowest height
        }
        else if (height == 2) {
            //set to medium height
        }
        else if (height == 3) {
            //set to high height
        }

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
        Robot.drivetrain.omniDrive(new Vector2D(0, -10), 0);
        //TODO: Set elevator height back

        isFinished = true;
    }

    @Override
    protected void execute() {
        if (Teleop.joystick.getBumperPressed(GenericHID.Hand.kLeft)) {
            this.curveLeft = true;
        }

        if (Teleop.joystick.getBumperPressed(GenericHID.Hand.kRight)) {
            this.curveRight = true;
        }

        if (Teleop.joystick.getAButtonPressed()) {
            this.height = 1;
        }

        if (Teleop.joystick.getXButtonPressed()) {
            this.height = 2;
        }

        if (Teleop.joystick.getYButtonPressed()) {
            this.height = 3;
        }

        if (Teleop.joystick.getBButtonPressed()) {
            cancel();
        }
    }

    @Override
    protected boolean isFinished() {
        return isFinished;
    }
}


