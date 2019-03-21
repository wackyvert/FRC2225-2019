package frc.team2225.robot.subsystem;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class RollerIntake extends Subsystem {
    VictorSP left;
    VictorSP right;
    Relay spike;
    private static final double outputLevel = 0.4;

    public RollerIntake(int left, int right, int wincher) {
        this.left = new VictorSP(left);
        this.left.setInverted(false);
        this.right = new VictorSP(right);
        this.right.setInverted(true);
        this.spike = new Relay(wincher, Relay.Direction.kBoth);
    }

    @Override
    protected void initDefaultCommand() {

    }

    public void grab() {
        right.set(outputLevel);
        left.set(outputLevel);
    }

    public void push() {
        right.set(-1);
        left.set(-1);
    }

    public void stopGrab() {
        left.set(0);
        right.set(0);
    }

    public void lift() {
        spike.set(Value.kForward);
    }

    public void drop() {
        spike.set(Value.kReverse);
    }

    public void stopLift() {
        spike.set(Value.kOff);
    }
}
