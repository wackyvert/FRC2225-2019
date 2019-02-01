package frc.team2225.robot.command;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class HookActuate extends Subsystem {
    
    TalonSRX motor;
    final double speed = 1;
    
    HookActuate(TalonSRX t){
        motor = t;
    }
    
    public void move(double input){
        motor.set(ControlMode.PercentOutput, Math.signum(input));
    }
    
    
    
    @Override
    protected void initDefaultCommand() {
    
    }
}
