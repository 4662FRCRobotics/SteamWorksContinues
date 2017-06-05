package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;
import org.usfirst.frc.team4662.robot.subsystems.VisionSystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Snappy extends Command {

    public Snappy() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.visionSystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.visionSystem.takePic();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
