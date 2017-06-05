package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveCurve extends Command {

	private double m_dAngle;
	private double m_dTimeout;
	private double m_dThrottle;
	private double m_dRotation;
	
    public DriveCurve(double angle, double throttle, double timeout) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSystem);
    	m_dAngle = -angle;
        m_dThrottle = -throttle;
        m_dTimeout = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(m_dTimeout);
    	Robot.driveSystem.resetGyro();
    	m_dRotation = m_dThrottle * ((Math.abs(m_dAngle))/ m_dAngle);
    	//sign of throttle
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveSystem.ArcadeDrive(m_dRotation, m_dThrottle);
    	//ROTATION SPEED, FORWARD OR BACKWARD
    	//direction, throttle
    	//direction = angle * throttle
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSystem.ArcadeDrive(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
