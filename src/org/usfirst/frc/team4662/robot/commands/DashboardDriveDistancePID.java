package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DashboardDriveDistancePID extends Command {

	private double m_dDistance;
	
    public DashboardDriveDistancePID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSystem);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	m_dDistance = Robot.driveSystem.dashboardFetch();
    	Robot.driveSystem.initEncoder(m_dDistance);
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
