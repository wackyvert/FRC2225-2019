package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team2225.robot.Robot;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;
import frc.team2225.robot.subsystem.Elevator.Level;

public class Teleop extends Command {
    private static XboxController joystick = new XboxController(0);

    /*private static ShuffleboardLayout layout = Robot.debugTab.getLayout("Joystick", BuiltInLayouts.kList.getLayoutName());
    private static NetworkTableEntry yOutput = layout.add("Joystick Y", 0).getEntry();
    private static NetworkTableEntry xOutput = layout.add("Joystick X", 0).getEntry();
    private static NetworkTableEntry rotateOutput = layout.add("Joystick Rot", 0).getEntry();*/
    boolean rightTriggerPrevious = false;
    boolean leftTriggerPrevious = false;
    Level currentLevel = Level.BOT_BALL;

    public Teleop() {
        requires(Robot.drivetrain);
        requires(Robot.elevator);
        requires(Robot.rollerIntake);
    }

    @Override
    protected void execute() {
        Vector2D translate = new Vector2D(joystick.getX(Hand.kLeft), -joystick.getY(Hand.kLeft));
        translate.transformComponents(ScaleInputs::scaleInputs);
        double rotate = joystick.getX(Hand.kRight);
        rotate = ScaleInputs.scaleInputs(rotate, 0.2, 0.15, 4);
        Robot.drivetrain.omniDrive(translate, rotate);

        double elevate = ScaleInputs.scaleInputs(joystick.getTriggerAxis(Hand.kRight) - joystick.getTriggerAxis(Hand.kLeft),
                0.02, 0.5, 2.0);
        if (elevate != 0) {
            Robot.elevator.setOutput(elevate);
        } else if (Robot.elevator.wasOutputSetManual()) {
            Robot.elevator.setOutput(0);
        }

        if (joystick.getAButton()) {
            Robot.rollerIntake.grab();
        } else if (joystick.getBButton()) {
            Robot.rollerIntake.push();
        } else {
            Robot.rollerIntake.stopGrab();
        }

        if (joystick.getYButton()) {
            Robot.rollerIntake.lift();
        } else if (joystick.getXButton()) {
            Robot.rollerIntake.drop();
        } else {
            Robot.rollerIntake.stopLift();
        }

        if (joystick.getBackButton()) {
            Robot.elevator.reset();
        }

        /*if (joystick.getBumperPressed(Hand.kLeft)) {
            Robot.elevator.setPosition(Level.changeLevel(currentLevel, true, false));
        } else if (joystick.getBumperPressed(Hand.kRight)) {
            Robot.elevator.setPosition(Level.changeLevel(currentLevel, true, true));
        } else if (false) {
            Robot.elevator.setPosition(Level.changeLevel(currentLevel, false, false));
        } else if (false) {
            Robot.elevator.setPosition(Level.changeLevel(currentLevel, false, true));
        }*/
    }

    private boolean getTrigger(Hand hand) {
        return joystick.getTriggerAxis(hand) > 0.5;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
