package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.team2225.robot.Robot;

public class Elevator extends Subsystem {

    // Units: counts / 100ms
    public static final int maxVelocity = 100;

    public static final double fGain = 1023.0 / maxVelocity;

    // Cruise Velocity = Max Velocity * 30%
    public static final int cruiseVelocity = 30;

    //Accelerate to cruise in 0.5 second
    public static final int acceleration = cruiseVelocity / 2;

    public enum Level {
        TOP(0), MID(0), BOT(0);

        double[] level;

        Level(double level) {
            this.level = new double[]{level};
        }
    }

    private TalonSRX motor;
    private double kP = 0, kI = 0, kD = 0;

    private ShuffleboardLayout pidLayout = Robot.debugTab.getLayout("Elevator PID");
    private ShuffleboardLayout levelLayout = Robot.debugTab.getLayout("Elevator Levels");

    private NetworkTableEntry pChooser = pidLayout.add("kP", 0).getEntry();
    private NetworkTableEntry iChooser = pidLayout.add("kI", 0).getEntry();
    private NetworkTableEntry dChooser = pidLayout.add("kD", 0).getEntry();

    private NetworkTableEntry topSet = levelLayout.add("Top Level", 0).getEntry();
    private NetworkTableEntry midSet = levelLayout.add("Middle Level", 0).getEntry();
    private NetworkTableEntry botSet = levelLayout.add("Bottom Level", 0).getEntry();
    private NetworkTableEntry currentSet = levelLayout.add("Current Level", 0).getEntry();


    @Override
    protected void initDefaultCommand() {

    }

    @Override
    public void periodic() {
        currentSet.setDouble(motor.getSelectedSensorPosition());
    }

    public void setPosition(Level level) {
        motor.set(ControlMode.Position, level.level[0]);
    }

    public Elevator(int port) {
        motor = new TalonSRX(port);
        motor.configFactoryDefault();
        motor.selectProfileSlot(0, 0);
        motor.config_kF(0, fGain);
        motor.config_IntegralZone(0, 0);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configClearPositionOnLimitR(true, 0);

        pChooser.addListener(v -> motor.config_kP(0, v.value.getDouble()), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        iChooser.addListener(v -> motor.config_kI(0, v.value.getDouble()), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        dChooser.addListener(v -> motor.config_kD(0, v.value.getDouble()), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

        topSet.addListener(v -> Level.TOP.level[0] = v.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        midSet.addListener(v -> Level.MID.level[0] = v.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        botSet.addListener(v -> Level.BOT.level[0] = v.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
    }
}
