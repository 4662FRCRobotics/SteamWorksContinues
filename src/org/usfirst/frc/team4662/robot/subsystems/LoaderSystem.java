package org.usfirst.frc.team4662.robot.subsystems;

import org.usfirst.frc.team4662.robot.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LoaderSystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private SpeedController Controller1;
	
	private double m_dLoadSpeed;
	private double m_dUnloadSpeed;
	
	public LoaderSystem() {
		Controller1 = new TalonSRX(RobotMap.loaderMotor);
		m_dLoadSpeed = -1.0;
		m_dUnloadSpeed = 0.2;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void loaderLoad(double speed) {
    	Controller1.set(speed);
    	logDashboard();
    }
    
    public void loaderLoadMidMan() {
    	loaderLoad(m_dLoadSpeed);
    }
    
    public void loaderStop() {
    	loaderLoad(0);
    }
    
    public void loaderTakeItBackNowYall() {
    	loaderLoad(m_dUnloadSpeed);
    }
    
    public double loaderSpeed() {
    	return Controller1.get();
    }
    
    @SuppressWarnings("deprecation")
	public void logDashboard() {
    	SmartDashboard.putNumber("LoaderSpeed", loaderSpeed());
    }
}