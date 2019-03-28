package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;

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

    /*private ShuffleboardLayout pidLayout = Robot.debugTab.getLayout("Elevator PID", BuiltInLayouts.kList.getLayoutName());
    private ShuffleboardLayout levelLayout = Robot.debugTab.getLayout("Elevator Levels", BuiltInLayouts.kList.getLayoutName());

    private NetworkTableEntry pChooser = pidLayout.add("kP", 0).getEntry();
    private NetworkTableEntry iChooser = pidLayout.add("kI", 0).getEntry();
    private NetworkTableEntry dChooser = pidLayout.add("kD", 0).getEntry();

    private NetworkTableEntry ballTop = levelLayout.add("Top Ball Level", 0).getEntry();
    private NetworkTableEntry hatchTop = levelLayout.add("Top Hatch Level", 0).getEntry();
    private NetworkTableEntry ballMid = levelLayout.add("Middle Ball Level", 0).getEntry();
    private NetworkTableEntry hatchMid = levelLayout.add("Middle Hatch Level", 0).getEntry();
    private NetworkTableEntry ballBot = levelLayout.add("Bottom Ball Level", 0).getEntry();
    private NetworkTableEntry hatchBot = levelLayout.add("Bottom Hatch Level", 0).getEntry();
    private NetworkTableEntry currentSet = levelLayout.add("Current Level", 0).getEntry();
    private NetworkTableEntry currentLevel = levelLayout.add("Enum Level", "Unknown").getEntry();*/


    @Override
    protected void initDefaultCommand() {

    }

    @Override
    public void periodic() {
//        currentSet.setDouble(motor.getSelectedSensorPosition());
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
        motor.set(ControlMode.PercentOutput, output * 0.7);
    }

    private void initEnum() {
/*        TOP_BALL.level[0] = ballTop;
        MID_BALL.level[0] = ballMid;
        BOT_BALL.level[0] = ballBot;
        TOP_HATCH.level[0] = hatchTop;
        MID_HATCH.level[0] = hatchMid;
        BOT_HATCH.level[0] = hatchBot;*/
    }

    public Elevator(int port) {
        initEnum();

        motor = new TalonSRX(port);
        motor.configFactoryDefault();
        motor.configOpenloopRamp(0.5);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
        motor.selectProfileSlot(0, 0);
        motor.config_kF(0, fGain);
        motor.config_IntegralZone(0, 0);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configClearPositionOnLimitR(true, 0);

//        pChooser.addListener(v -> motor.config_kP(0, v.value.getDouble()), Robot.updateFlags);
//        iChooser.addListener(v -> motor.config_kI(0, v.value.getDouble()), Robot.updateFlags);
//        dChooser.addListener(v -> motor.config_kD(0, v.value.getDouble()), Robot.updateFlags);
    }
}
