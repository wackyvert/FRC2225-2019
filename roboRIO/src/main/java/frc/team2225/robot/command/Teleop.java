package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.InterruptableSensorBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2225.robot.Robot;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;

public class Teleop extends Command {
    static XboxController joystick;

    public Teleop() {
        requires(Robot.drivetrain);
        joystick = new XboxController(0);
    }

    @Override
    protected void execute() {
        Vector2D translate = new Vector2D(joystick.getX(GenericHID.Hand.kLeft), -joystick.getY(GenericHID.Hand.kLeft));
        translate.transformComponents(ScaleInputs::scaleInputs);
        double rotate = joystick.getX(GenericHID.Hand.kRight);
        rotate = ScaleInputs.scaleInputs(rotate);
        Robot.drivetrain.omniDrive(translate, rotate);

        if (joystick.getY(GenericHID.Hand.kLeft) < 0) {
            Robot.rollerIntake.grab(true);
        }

        if (joystick.getY(GenericHID.Hand.kLeft) == 0) {
            Robot.rollerIntake.grab(false);
            Robot.rollerIntake.push(false);
        }

        if (joystick.getY(GenericHID.Hand.kLeft) > 0) {
            Robot.rollerIntake.push(true);
        }

        if (joystick.getAButtonPressed()) {
            BallPlace bP = new BallPlace(false,false, 1);
            bP.start();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
