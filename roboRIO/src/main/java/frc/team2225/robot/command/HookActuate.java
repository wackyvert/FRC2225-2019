package frc.team2225.robot.command;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

public class HookActuate extends Subsystem {
    
    VictorSP motor;
    final double speed = 1;
    
    HookActuate(int port){
        motor = new VictorSP(port);
    }
    
    public void move(double input){
        motor.set(Math.signum(input * speed));
    }
    
    
    
    @Override
    protected void initDefaultCommand() {
    
    }
}
