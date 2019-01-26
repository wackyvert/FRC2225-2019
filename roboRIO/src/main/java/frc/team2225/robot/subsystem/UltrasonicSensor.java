package frc.team2225.robot.subsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class UltrasonicSensor extends Subsystem {
    AnalogInput sensorPort;
    final double scalar = 1;

    public UltrasonicSensor(int portNumber) {
        sensorPort = new AnalogInput(0);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public double getDistance() {
        return scalar * sensorPort.getAverageVoltage();
    }
}
