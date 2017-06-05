package org.usfirst.frc.team4662.robot.subsystems;

import org.usfirst.frc.team4662.robot.RobotMap;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShooterSystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private SpeedController Controller1;
	
	private Servo GateServo;
	
	private double m_dShooterSpeed;
	private double m_dServoAngleClosed;
	private double m_dServoAngleOpen;
	
	public ShooterSystem() {
		
		Controller1 = new VictorSP(RobotMap.shooterMotor1);
		
		GateServo = new Servo(RobotMap.gateServo);
		
		m_dShooterSpeed = -1.0;
		m_dServoAngleClosed = 90;
		m_dServoAngleOpen = 0;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void ballShooter(double speed) {
    	
    	Controller1.set(speed);
    	
    	logDashboard();
    }
    
    public void shooterGo() {
    	ballShooter(m_dShooterSpeed);
    }
    
    public void shooterStop() {
    	ballShooter(0);
    }
    
    public double shooterSpeed() {
    	return m_dShooterSpeed;
    }
    
    @SuppressWarnings("deprecation")
	public void logDashboard() {
    	SmartDashboard.putNumber("ShooterSpeed", Controller1.get());
    	SmartDashboard.putNumber("ServoAngle", GateServo.getAngle());
    }
    
    public void servoMove(double angle){
    	GateServo.setAngle(angle);
    }
    
    public void servoClose(){
    	servoMove(m_dServoAngleClosed);
    }
    
    public void servoOpen(){
    	servoMove(m_dServoAngleOpen);
    }
    
}

