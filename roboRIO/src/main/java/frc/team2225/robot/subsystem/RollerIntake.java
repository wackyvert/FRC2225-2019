package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class RollerIntake extends Subsystem {
    TalonSRX left;
    TalonSRX right;

    public RollerIntake(int left, int right) {
        this.left = new TalonSRX(left);
        this.right = new TalonSRX(right);
    }

    @Override
    protected void initDefaultCommand() {

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
}
