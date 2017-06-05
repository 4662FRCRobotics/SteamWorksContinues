package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DashboardGyroRotatePID extends Command {

	private double m_dAngle;
	
    public DashboardGyroRotatePID() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSystem);
    	
    }
    protected void initialize() {
    // Called just before this Command runs the first time
    	m_dAngle = Robot.driveSystem.dashboardGyroFetch();
    	Robot.driveSystem.turnToAngle(m_dAngle);
    	
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
