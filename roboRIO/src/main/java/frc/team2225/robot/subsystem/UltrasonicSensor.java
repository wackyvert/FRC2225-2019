package frc.team2225.robot.subsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class UltrasonicSensor extends Subsystem {
    ShuffleboardTab tab = Shuffleboard.getTab("Ultrasonic Sensor");
    NetworkTableEntry distance = tab.add("Distance", 0).getEntry();
    AnalogInput sensorPort;
    final double scalar = 50 * 2.54;

    public UltrasonicSensor(int portNumber) {
        sensorPort = new AnalogInput(0);
    }

    @Override
    protected void initDefaultCommand() {

    }

    @Override
    public void periodic() {
        distance.setDouble(getDistance());
    }

    public double getDistance() {
        return (sensorPort.getAverageVoltage() - .07) * scalar;
    }
}
