package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveDistancePID extends Command {

	private double m_dDistance;
	private double m_dThrottle;
	
    public DriveDistancePID(double distance, double throttle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSystem);
    	m_dDistance = distance;
    	m_dThrottle = throttle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Robot.driveSystem.initEncoder(m_dDistance, m_dThrottle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.driveSystem.encoderOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSystem.ArcadeDrive(0, 0);
    	Robot.driveSystem.disableEncoder();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
