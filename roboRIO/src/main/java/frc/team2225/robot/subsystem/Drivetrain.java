package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.team2225.robot.Robot;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;
import frc.team2225.robot.command.Teleop;

import java.util.function.BiConsumer;

import static frc.team2225.robot.subsystem.Drivetrain.Position.*;

public class Drivetrain extends Subsystem {

    public enum Position {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
    }
    public static final Vector2D frontLeftVec = new Vector2D(Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D frontRightVec = new Vector2D(-Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D backLeftVec = new Vector2D(-Math.sqrt(2) / 2, Math.sqrt(2) / 2);
    public static final Vector2D backRightVec = new Vector2D(Math.sqrt(2) / 2, Math.sqrt(2) / 2);

    public static final double _wheelCircumferenceCm = 6 * 2.54 * Math.PI;
    public static final double _motorRotsPerWheelRot = 16;
    public static final int _countsPerMotorRot = 40;

    public static final int deadZone = 100;

    int resetTargetRot;
    double targetRot;

    PidCallback pidWrite = new PidCallback();

    // Units: counts / 100ms
    public static final int maxVelocity = 100;

    public static final double fGain = 1023.0 / maxVelocity;

    // Cruise Velocity = Max Velocity * 30%
    public static final int cruiseVelocity = 30;

    //Accelerate to cruise in 1 second
    public static final int acceleration = cruiseVelocity;

    private ShuffleboardLayout drivePid = Robot.debugTab.getLayout("Drivetrain PID");
    private ShuffleboardLayout gyroPid = Robot.debugTab.getLayout("Gyro PID");
    private ShuffleboardLayout drivetrainOutputs = Robot.debugTab.getLayout("Drivetrain Outputs");

    private NetworkTableEntry[] motorOutputs = new NetworkTableEntry[4];

    private NetworkTableEntry pChooser = drivePid.add("kP", 0).getEntry();
    private NetworkTableEntry iChooser = drivePid.add("kI", 0).getEntry();
    private NetworkTableEntry dChooser = drivePid.add("kD", 0).getEntry();
    private NetworkTableEntry fChooser = drivePid.add("kF", fGain).getEntry();
    private NetworkTableEntry izChooser = drivePid.add("iZone", 0).getEntry();

    private NetworkTableEntry gpChooser = gyroPid.add("kP", 0).getEntry();
    private NetworkTableEntry giChooser = gyroPid.add("kI", 0).getEntry();
    private NetworkTableEntry gdChooser = gyroPid.add("kD", 0).getEntry();

    private void setMotorParam(NetworkTableEntry slot, BiConsumer<TalonSRX, Double> method) {
        slot.addListener(change -> {
            for (TalonSRX motor : motors) {
                method.accept(motor, change.value.getDouble());
            }
        }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
    }

    public TalonSRX[] motors;
    public ADXRS450_Gyro gyro;
    final PIDController turnController;
    public Drivetrain(int frontLeft, int frontRight, int backLeft, int backRight, SPI.Port gyro) {
        motors = new TalonSRX[4];
        motors[FRONT_LEFT.ordinal()] = new TalonSRX(frontLeft);
        motors[FRONT_RIGHT.ordinal()] = new TalonSRX(frontRight);
        motors[BACK_LEFT.ordinal()] = new TalonSRX(backLeft);
        motors[BACK_RIGHT.ordinal()] = new TalonSRX(backRight);
        for (Position position : Position.values()) {
            motorOutputs[position.ordinal()] = drivetrainOutputs.add(position.name() + " Output", 0).getEntry();
        }
        this.gyro = new ADXRS450_Gyro(gyro);
        setMotorParam(pChooser, (m, p) -> m.config_kP(0, p));
        setMotorParam(iChooser, (m, i) -> m.config_kI(0, i));
        setMotorParam(dChooser, (m, d) -> m.config_kD(0, d));
        setMotorParam(fChooser, (m, f) -> m.config_kF(0, f));
        setMotorParam(izChooser, (m, iz) -> m.config_IntegralZone(0, iz.intValue()));
        turnController = new PIDController(0, 0, 0, this.gyro, pidWrite);
        gpChooser.addListener(v -> turnController.setP(v.value.getDouble()), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        giChooser.addListener(v -> turnController.setI(v.value.getDouble()), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        gdChooser.addListener(v -> turnController.setD(v.value.getDouble()), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);

        for (TalonSRX motor : motors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
            motor.configNominalOutputForward(0.0);
            motor.configNominalOutputReverse(0.0);
            motor.configPeakOutputForward(1);
            motor.configPeakOutputReverse(-1);

            motor.selectProfileSlot(0, 0);
            motor.config_IntegralZone(0, 0);
            motor.config_kF(0, fGain);

            motor.configMotionCruiseVelocity(cruiseVelocity);
            motor.configMotionAcceleration(acceleration);
        }

        targetRot = 0;
        motorOf(FRONT_RIGHT).setInverted(true);
        motorOf(BACK_RIGHT).setInverted(true);
        motorOf(FRONT_LEFT).setInverted(false);
        motorOf(BACK_LEFT).setInverted(false);
    }

    public TalonSRX motorOf(Position position) {
        return motors[position.ordinal()];
    }

    /**
     * Drive the robot by setting the output voltage
     *
     * @param translate A vector which describes the desired movement direction. Units are percent output [0, 1]
     * @param rotateIn  The desired rotation amount (positive is clockwise)
     */
    public void omniDrive(Vector2D translate, double rotateIn) {
        // TODO: test and use rotate instead of rotateIn
        translate.mapSquareToDiamond().divide(Math.sqrt(2) / 2);
        final double p = 1.0/150.0;
        double fr, fl, br, bl;
        double rotate = 0;
        if(rotateIn != 0) {
            resetTargetRot = 10;
            rotate = -rotateIn;
        }
        if(resetTargetRot > 0) {
            targetRot = gyro.getAngle();
            resetTargetRot--;
        }
        if(rotateIn == 0 && resetTargetRot <= 0) {
            rotate = Math.max(-1, Math.min(pidWrite.output ,1));
        }
        fl = translate.dot(frontLeftVec);
        fr = translate.dot(frontRightVec);
        bl = translate.dot(backLeftVec);
        br = translate.dot(backRightVec);


        fl = ScaleInputs.padMinValue(rotateIn, fl, false) + rotateIn;
        fr = ScaleInputs.padMinValue(rotateIn, fr, false) - rotateIn;
        bl = ScaleInputs.padMinValue(rotateIn, bl, false) + rotateIn;
        br = ScaleInputs.padMinValue(rotateIn, br, false) - rotateIn;

        setMotorVoltage(fl, fr, bl, br);
    }

    private int cmToCounts(double cm) {
        return (int)(cm / _wheelCircumferenceCm * _motorRotsPerWheelRot * _countsPerMotorRot);
    }

    /**
     * Used to drive the robot autonomously a certain distance
     * @param v The desired distance of translation (units are centimeters)
     * @return A CheckPosition object that can be used to see if the movement has completed
     */
    public CheckPosition translate(Vector2D v){
        int fl = motorOf(FRONT_LEFT).getSelectedSensorPosition() + cmToCounts(v.dot(frontLeftVec));
        int fr = motorOf(FRONT_RIGHT).getSelectedSensorPosition() + cmToCounts(v.dot(frontRightVec));
        int bl = motorOf(BACK_LEFT).getSelectedSensorPosition() + cmToCounts(v.dot(backLeftVec));
        int br = motorOf(BACK_RIGHT).getSelectedSensorPosition() + cmToCounts(v.dot(backRightVec));
        motorOf(FRONT_LEFT).set(ControlMode.Position, fl);
        motorOf(FRONT_RIGHT).set(ControlMode.Position, fr);
        motorOf(BACK_LEFT).set(ControlMode.Position, bl);
        motorOf(BACK_RIGHT).set(ControlMode.Position, br);
        return new CheckPosition(fl, fr, bl, br, deadZone);
    }

    public void setMotorVoltage(double fl, double fr, double bl, double br) {
        motorOf(FRONT_LEFT).set(ControlMode.PercentOutput, fl);
        motorOutputs[FRONT_LEFT.ordinal()].setDouble(fl);
        motorOf(FRONT_RIGHT).set(ControlMode.PercentOutput, fr);
        motorOutputs[FRONT_RIGHT.ordinal()].setDouble(fr);
        motorOf(BACK_LEFT).set(ControlMode.PercentOutput, bl);
        motorOutputs[BACK_LEFT.ordinal()].setDouble(bl);
        motorOf(BACK_RIGHT).set(ControlMode.PercentOutput, br);
        motorOutputs[BACK_RIGHT.ordinal()].setDouble(br);
    }

    /**
     * Stop the robot
     */
    public void stop() {
        omniDrive(Vector2D.zero(), 0);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Teleop());
    }

    public class CheckPosition{

        private final int deadZone;
        int fl, fr, bl, br;

        CheckPosition(int fl, int fr, int bl, int br, int deadZone){
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
            this.deadZone = deadZone;
        }

        public boolean isDone(){
            if(Math.abs(fl - motorOf(FRONT_LEFT).getSelectedSensorPosition()) <= deadZone){
                if(Math.abs(fr - motorOf(FRONT_RIGHT).getSelectedSensorPosition()) <= deadZone){
                    if(Math.abs(bl - motorOf(BACK_LEFT).getSelectedSensorPosition()) <= deadZone){
                        return Math.abs(br - motorOf(BACK_RIGHT).getSelectedSensorPosition()) <= deadZone;
                    }
                }
            }
            return false;
        }

        @Override
        public String toString() {
            return "fl: " + (fl - motorOf(FRONT_LEFT).getSelectedSensorPosition()) +
                    ", fr: " + (fr - motorOf(FRONT_RIGHT).getSelectedSensorPosition()) +
                    ", bl: " + (bl - motorOf(BACK_LEFT).getSelectedSensorPosition()) +
                    ", br: " + (br - motorOf(BACK_RIGHT).getSelectedSensorPosition());
        }
    }

    public class PidCallback implements PIDOutput {
        public double output;
        @Override
        public void pidWrite(double output) {
            this.output = output;
        }
    }

}
