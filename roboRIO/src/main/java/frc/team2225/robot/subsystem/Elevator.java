package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Elevator extends Subsystem {
    public enum Level {
        TOP(0), MID(0), BOT(0);

        double[] level;

        Level(double level) {
            this.level = new double[]{level};
        }
    }

    private TalonSRX motor;
    private double kP = 0, kI = 0, kD = 0;

    private ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    private NetworkTableEntry pChooser = tab.add("kP", 0).getEntry();
    private NetworkTableEntry iChooser = tab.add("kI", 0).getEntry();
    private NetworkTableEntry dChooser = tab.add("kD", 0).getEntry();

    private NetworkTableEntry topSet = tab.add("Top Level", 0).getEntry();
    private NetworkTableEntry midSet = tab.add("Middle Level", 0).getEntry();
    private NetworkTableEntry botSet = tab.add("Bottom Level", 0).getEntry();
    private NetworkTableEntry currentSet = tab.add("Current Level", 0).getEntry();


    @Override
    protected void initDefaultCommand() {

    }

    @Override
    public void periodic() {
        if (kP != pChooser.getDouble(0)) {
            kP = pChooser.getDouble(0);
            motor.config_kP(0, kP);
        }

        if (kI != iChooser.getDouble(0)) {
            kI = iChooser.getDouble(0);
            motor.config_kI(0, kI);
        }

        if (kD != dChooser.getDouble(0)) {
            kD = dChooser.getDouble(0);
            motor.config_kD(0, kD);
        }

        currentSet.setDouble(motor.getSelectedSensorPosition());
    }

    public void setPosition(Level level) {
        motor.set(ControlMode.Position, level.level[0]);
    }

    public Elevator(int port) {
        motor = new TalonSRX(port);
        motor.configFactoryDefault();
        motor.selectProfileSlot(0, 0);
        motor.config_kP(0, kP);
        motor.config_kI(0, kI);
        motor.config_kD(0, kD);
        motor.config_IntegralZone(0, 0);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configClearPositionOnLimitR(true, 0);

        topSet.addListener(v -> Level.TOP.level[0] = v.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        midSet.addListener(v -> Level.MID.level[0] = v.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        botSet.addListener(v -> Level.BOT.level[0] = v.value.getDouble(), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
    }
}
