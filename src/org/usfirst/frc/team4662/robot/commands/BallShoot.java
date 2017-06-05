package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BallShoot extends Command {
	
	private boolean isAutonomous;
	private double m_dTimeOut;

    public BallShoot() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.boilerLoader);
    	isAutonomous = false;
    	m_dTimeOut = 0;
    }
    public BallShoot(double TO) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.boilerLoader);
    	isAutonomous = true;
    	m_dTimeOut = TO;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(m_dTimeOut);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.boilerLoader.shooterGo();
    	Robot.boilerLoader.servoOpen();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isAutonomous && isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.boilerLoader.shooterStop();
    	Robot.boilerLoader.servoClose();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
