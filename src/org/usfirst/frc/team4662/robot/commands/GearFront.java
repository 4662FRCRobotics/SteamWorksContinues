package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GearFront extends Command {

    public GearFront() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveSystem);
        requires(Robot.visionSystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveSystem.GearForward();
    	Robot.visionSystem.GearCam();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.driveSystem.isToggled();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
