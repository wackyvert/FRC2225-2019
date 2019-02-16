package frc.team2225.robot.command;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.team2225.robot.Robot;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;

public class Teleop extends Command {
    private static XboxController joystick = new XboxController(0);

    private static ShuffleboardLayout layout = Robot.debugTab.getLayout("Joystick");
    private static NetworkTableEntry yOutput = layout.add("Joystick Y", 0).getEntry();
    private static NetworkTableEntry xOutput = layout.add("Joystick X", 0).getEntry();

    public Teleop() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void execute() {
        Vector2D translate = new Vector2D(joystick.getX(GenericHID.Hand.kLeft), -joystick.getY(GenericHID.Hand.kLeft));
        translate.transformComponents(ScaleInputs::scaleInputs);
        double rotate = joystick.getX(GenericHID.Hand.kRight);
        rotate = ScaleInputs.scaleInputs(rotate);
        Robot.drivetrain.omniDrive(translate, rotate);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
