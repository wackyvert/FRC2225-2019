package frc.team2225.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team2225.robot.ScaleInputs;
import frc.team2225.robot.Vector2D;
import frc.team2225.robot.command.Teleop;

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

    public TalonSRX[] motors;
    public ADXRS450_Gyro gyro;

    public Drivetrain(int frontLeft, int frontRight, int backLeft, int backRight, SPI.Port gyro) {
        motors = new TalonSRX[4];
        motors[FRONT_LEFT.ordinal()] = new TalonSRX(frontLeft);
        motors[FRONT_RIGHT.ordinal()] = new TalonSRX(frontRight);
        motors[BACK_LEFT.ordinal()] = new TalonSRX(backLeft);
        motors[BACK_RIGHT.ordinal()] = new TalonSRX(backRight);
        this.gyro = new ADXRS450_Gyro(gyro);
        targetRot = 0;
        motorOf(FRONT_RIGHT).setInverted(true);
        motorOf(BACK_RIGHT).setInverted(true);
        motorOf(FRONT_LEFT).setInverted(false);
        motorOf(BACK_LEFT).setInverted(false);

        for(TalonSRX motor : motors) {
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
            motor.configNominalOutputForward(0.0, 0);
            motor.configNominalOutputReverse(0.0, 0);
            motor.configPeakOutputForward(1, 0);
            motor.configPeakOutputReverse(-1, 0);
            motor.configClosedloopRamp(0.4, 0);
            motor.config_kP(0, 0.5, 0);
            motor.config_kI(0, 0, 0);
            motor.config_kD(0, 2, 0);
            motor.config_IntegralZone(0, 0, 0);
        }
    }

    public TalonSRX motorOf(Position position) {
        return motors[position.ordinal()];
    }

    public void omniDrive(Vector2D translate, double rotateIn) {
        translate.mapSquareToDiamond().divide(Math.sqrt(2) / 2);
        final double p = 1.0/150.0;
        double fr, fl, br, bl;
        fl = translate.dot(frontLeftVec);
        fr = translate.dot(frontRightVec);
        bl = translate.dot(backLeftVec);
        br = translate.dot(backRightVec);

        double rotate = 0;
        if(rotateIn != 0) {
            resetTargetRot = 10;
            rotate = -rotateIn;
        }
        if(resetTargetRot > 0) {
            targetRot = gyro.getAngle();
            resetTargetRot--;
        }
        if(rotateIn == 0) {
            double pTerm = resetTargetRot > 0 ? 0 : (gyro.getAngle() - targetRot) * p;
            rotate = pTerm;
            rotate = Math.max(-1, Math.min(rotate, 1));
        }
        fl = ScaleInputs.padMinValue(rotate, fl, false) + rotate;
        fr = ScaleInputs.padMinValue(rotate, fr, false) - rotate;
        bl = ScaleInputs.padMinValue(rotate, bl, false) + rotate;
        br = ScaleInputs.padMinValue(rotate, br, false) - rotate;

        setMotorVoltage(fl, fr, bl, br);

    }

    private int cmToCounts(double cm) {
        return (int)(cm / _wheelCircumferenceCm * _motorRotsPerWheelRot * _countsPerMotorRot);
    }

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
        motorOf(FRONT_RIGHT).set(ControlMode.PercentOutput, fr);
        motorOf(BACK_LEFT).set(ControlMode.PercentOutput, bl);
        motorOf(BACK_RIGHT).set(ControlMode.PercentOutput, br);
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

    }


}
