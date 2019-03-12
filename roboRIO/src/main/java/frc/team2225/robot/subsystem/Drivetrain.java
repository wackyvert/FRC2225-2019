package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.team2225.robot.Robot;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;
import frc.team2225.robot.command.Teleop;

import java.util.function.BiConsumer;

import static frc.team2225.robot.subsystem.Drivetrain.Position.*;

public class Drivetrain extends Subsystem {

    boolean gyroEnable;

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

    // Units: counts / 100ms
    public static final int maxVelocity = 100;

    public static final double fGain = 1023.0 / maxVelocity;

    // Cruise Velocity = Max Velocity * 30%
    public static final int cruiseVelocity = 30;

    //Accelerate to cruise in 1 second
    public static final int acceleration = cruiseVelocity;

    private NetworkTableEntry gyroVal = Shuffleboard.getTab("Main").add("Gyro Val", 0).getEntry();
    private NetworkTableEntry gyroOut = Shuffleboard.getTab("Main").add("Gyro Out", 0).getEntry();
    /*private NetworkTableEntry pOut = Shuffleboard.getTab("Main").add("P Out", 0).getEntry();
    private NetworkTableEntry dOut = Shuffleboard.getTab("Main").add("D Out", 0).getEntry();*/

    private NetworkTableEntry gyroEnableNet = Shuffleboard.getTab("Main").add("Gyro Enable", true).getEntry();

    @Override
    public void periodic() {
        gyroVal.setDouble(gyro.getAngle());
    }

    private void setMotorParam(NetworkTableEntry slot, BiConsumer<TalonSRX, Double> method) {
        slot.addListener(change -> {
            for (TalonSRX motor : motors) {
                method.accept(motor, change.value.getDouble());
            }
        }, Robot.updateFlags);
    }

    public TalonSRX[] motors;
    public ADXRS450_Gyro gyro;
    public Drivetrain(int frontLeft, int frontRight, int backLeft, int backRight, SPI.Port gyro) {
        gyroEnable = true;
        motors = new TalonSRX[4];
        motors[FRONT_LEFT.ordinal()] = new TalonSRX(frontLeft);
        motors[FRONT_RIGHT.ordinal()] = new TalonSRX(frontRight);
        motors[BACK_LEFT.ordinal()] = new TalonSRX(backLeft);
        motors[BACK_RIGHT.ordinal()] = new TalonSRX(backRight);
        /*for (Position position : Position.values()) {
            motorOutputs[position.ordinal()] = drivetrainOutputs.add(position.name() + " Output", 0).getEntry();
        }*/
        this.gyro = new ADXRS450_Gyro(gyro);
        /*Robot.debugTab.add("Gyro", this.gyro);
        DriverStation.reportWarning("Gyro: " + this.gyro.isConnected() + ", " + this.gyro.getAngle(), false);
        setMotorParam(pChooser, (m, p) -> m.config_kP(0, p));
        setMotorParam(iChooser, (m, i) -> m.config_kI(0, i));
        setMotorParam(dChooser, (m, d) -> m.config_kD(0, d));
        setMotorParam(fChooser, (m, f) -> m.config_kF(0, f));
        setMotorParam(izChooser, (m, iz) -> m.config_IntegralZone(0, iz.intValue()));*/
        /*gpChooser.addListener(v -> {
            turnController.setP(v.value.getDouble());
            DriverStation.reportWarning("P Updated " + v.value.getDouble(), false);
        }, Robot.updateFlags);
        giChooser.addListener(v -> turnController.setI(v.value.getDouble()), Robot.updateFlags);
        gdChooser.addListener(v -> turnController.setD(v.value.getDouble()), Robot.updateFlags);*/

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
        motorOf(FRONT_RIGHT).setInverted(false);
        motorOf(BACK_RIGHT).setInverted(false);
        motorOf(FRONT_LEFT).setInverted(false);
        motorOf(BACK_LEFT).setInverted(true);
    }

    public void setGyroEnable(boolean enable) {
        gyroEnable = enable;
        gyroEnableNet.setBoolean(enable);
    }

    public void toggleGyroEnable() {
        setGyroEnable(!gyroEnable);
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
        final double p = 0.025;
        final double d = 0.002;
        // TODO: test and use rotate instead of rotateIn
        translate.mapSquareToDiamond().divide(Math.sqrt(2) / 2);
        double fr, fl, br, bl;
        fl = translate.dot(frontLeftVec);
        fr = translate.dot(frontRightVec);
        bl = translate.dot(backLeftVec);
        br = translate.dot(backRightVec);

        boolean controllerInputIsPresent = Math.abs(rotateIn) > 0.1;
        double rotate = 0;
        if (controllerInputIsPresent) {
            resetTargetRot = 10;
            rotate = rotateIn;
        }
        if(resetTargetRot > 0) {
            targetRot = gyro.getAngle();
            resetTargetRot--;
        }
        if (!controllerInputIsPresent && gyroEnable) {
            double pTerm = resetTargetRot > 0 ? 0 : -(gyro.getAngle() - targetRot) * p;
//            pOut.setDouble(pTerm);
            double dTerm = gyro.getRate() * d;
//            dOut.setDouble(dTerm);
            dTerm = Math.copySign(Math.max(0, Math.abs(dTerm) - 0.1), dTerm);
            rotate = pTerm + dTerm;
            rotate = Math.max(-0.5, Math.min(rotate, 0.5));
            gyroOut.setDouble(rotate);
        }


        fl = ScaleInputs.padMinValue(rotateIn, fl, false) + rotate;
        fr = ScaleInputs.padMinValue(rotateIn, fr, false) - rotate;
        bl = ScaleInputs.padMinValue(rotateIn, bl, false) + rotate;
        br = ScaleInputs.padMinValue(rotateIn, br, false) - rotate;
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
//        motorOutputs[FRONT_LEFT.ordinal()].setDouble(fl);
        motorOf(FRONT_RIGHT).set(ControlMode.PercentOutput, fr);
//        motorOutputs[FRONT_RIGHT.ordinal()].setDouble(fr);
        motorOf(BACK_LEFT).set(ControlMode.PercentOutput, bl);
//        motorOutputs[BACK_LEFT.ordinal()].setDouble(bl);
        motorOf(BACK_RIGHT).set(ControlMode.PercentOutput, br);
//        motorOutputs[BACK_RIGHT.ordinal()].setDouble(br);
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

    public class PIDCallback implements PIDOutput {
        public double output;
        @Override
        public void pidWrite(double output) {
            this.output = output;
            DriverStation.reportWarning("PID Updated: " + output, false);
        }
    }

}
