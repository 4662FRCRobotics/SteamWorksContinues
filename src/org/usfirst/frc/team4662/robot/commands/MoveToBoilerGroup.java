package org.usfirst.frc.team4662.robot.commands;

import org.usfirst.frc.team4662.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToBoilerGroup extends CommandGroup {

	private double m_dSpeed;
	
    public MoveToBoilerGroup(double speed) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	requires(Robot.visionSystem);
    	requires(Robot.driveSystem);
    	requires(Robot.boilerLoader);
    	
    	m_dSpeed = speed;
    	
    	addSequential(new GyroRotatePID(Robot.visionSystem.getBoilerAngle(), m_dSpeed));
    	addSequential(new DriveDistancePID(Robot.visionSystem.getBoilerDistance(), m_dSpeed));
    	Robot.visionSystem.LightOff();
    	addSequential(new BallShoot(8));
    }
}
