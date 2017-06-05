package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GyroRotatePID extends Command {

	private double m_dAngle;
	private double m_dSpeed;
	
    public GyroRotatePID(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSystem);
    	m_dAngle = angle;
    	m_dSpeed = 0.6;
    }

    public GyroRotatePID(double angle, double speed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSystem);
    	m_dAngle = angle;
    	m_dSpeed = speed;
    }
    
    // Called just before this Command runs the first time
    protected void initialize() {
       
    	Robot.driveSystem.turnToAngle(m_dAngle, m_dSpeed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.driveSystem.gyroOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSystem.ArcadeDrive(0, 0);
    	Robot.driveSystem.disableGyro();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
