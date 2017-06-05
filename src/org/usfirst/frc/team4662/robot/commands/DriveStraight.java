package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command {

    public DriveStraight() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveSystem.keepHeading();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double throttle = 2 / (Robot.oi.driveStick.getThrottle() + 3.0);
    	Robot.driveSystem.stayTrue(Robot.oi.driveStick.getY() * throttle);
    	//currently .5 to 1
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveSystem.disableKeepAngle();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
