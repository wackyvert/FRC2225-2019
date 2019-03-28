package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Elevator extends Subsystem {

    // Units: counts / 100ms
    public static final int maxVelocity = 100;

    public static final double fGain = 1023.0 / maxVelocity;

    // Cruise Velocity = Max Velocity * 30%
    public static final int cruiseVelocity = 30;

    //Accelerate to cruise in 0.5 second
    public static final int acceleration = cruiseVelocity / 2;
    boolean manualOutput = false;

    public enum Level {
        BOT_HATCH(true), BOT_BALL(false), MID_HATCH(true), MID_BALL(false), TOP_HATCH(true), TOP_BALL(false);

        NetworkTableEntry[] level;
        boolean isHatch;

        Level(boolean isHatch) {
            this.isHatch = isHatch;
            this.level = new NetworkTableEntry[1];
        }

        public static Level changeLevel(Level currentLevel, boolean isHatch, boolean moveUp) {
            int delta = moveUp ? 1 : -1;
            for (int ord = currentLevel.ordinal() + delta; ord < Level.values().length && ord >= 0; ord += delta) {
                Level next = Level.values()[ord];
                if (next.isHatch == isHatch)
                    return next;
            }
            return currentLevel;
        }
    }

    private TalonSRX motor;
    private double kP = 0, kI = 0, kD = 0;

/*
    private NetworkTableEntry pChooser = pidLayout.add("kP", 0).getEntry();
    private NetworkTableEntry iChooser = pidLayout.add("kI", 0).getEntry();
    private NetworkTableEntry dChooser = pidLayout.add("kD", 0).getEntry();
*/

    private NetworkTableEntry level = Shuffleboard.getTab("Main").add("Elevator Level", 0).getEntry();


    @Override
    protected void initDefaultCommand() {

    }

    public void reset() {
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        level.setDouble(motor.getSelectedSensorPosition());
    }

    public boolean wasOutputSetManual() {
        return manualOutput;
    }

    public void zero() {
        motor.setSelectedSensorPosition(0);
    }

    public void setPosition(Level level) {
        manualOutput = false;
//        currentLevel.setString(level.name());
        motor.set(ControlMode.Position, level.level[0].getValue().getDouble());
    }

    public void setOutput(double output) {
        manualOutput = true;
//        currentLevel.setString("Manual");
        motor.set(ControlMode.PercentOutput, output);
    }

    public Elevator(int port) {
        motor = new TalonSRX(port);
        motor.configFactoryDefault();
        motor.configOpenloopRamp(0.5);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
        motor.selectProfileSlot(0, 0);
        motor.config_kF(0, fGain);
        motor.config_IntegralZone(0, 0);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        motor.setSensorPhase(true);
        motor.configClearPositionOnLimitR(true, 0);

//        pChooser.addListener(v -> motor.config_kP(0, v.value.getDouble()), Robot.updateFlags);
//        iChooser.addListener(v -> motor.config_kI(0, v.value.getDouble()), Robot.updateFlags);
//        dChooser.addListener(v -> motor.config_kD(0, v.value.getDouble()), Robot.updateFlags);
    }
}
