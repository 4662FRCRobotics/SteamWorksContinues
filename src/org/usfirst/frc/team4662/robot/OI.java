package org.usfirst.frc.team4662.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4662.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	public Joystick driveStick = new Joystick(0);
	public JoystickButton ToggleFront = new JoystickButton(driveStick,9); 
	//public JoystickButton Take5 = new JoystickButton(driveStick, 5);
	public JoystickButton ToggleVision = new JoystickButton(driveStick, 7);
	public JoystickButton Climb = new JoystickButton(driveStick, 2);
	public JoystickButton Shoot = new JoystickButton(driveStick, 1);
	public JoystickButton Load = new JoystickButton(driveStick, 3); 
	public JoystickButton Unload = new JoystickButton(driveStick, 4); 
	public JoystickButton StayTrue = new JoystickButton(driveStick, 5);
	
	public OI()  {
		ToggleFront.whenPressed(new SwitchFront());	
		//Take5.whenPressed(new WaitForIt(5));
		ToggleVision.whenPressed(new VisionToggle());
		Climb.whileHeld(new ClimbRope());
		Shoot.whileHeld(new BallShoot());
		Load.whileHeld(new LoadLoad());
		Unload.whileHeld(new LoaderUnload());
		StayTrue.whileHeld(new DriveStraight());
		
		SmartDashboard.putData("InterruptPID", new InterruptPID());
		SmartDashboard.putData("DashboardDrivePID", new DashboardDriveDistancePID());
		SmartDashboard.putData("DashboardGyroPID", new DashboardGyroRotatePID());
		SmartDashboard.putData("ResetBothEncoder", new ResetBothEncoder());
		SmartDashboard.putData("CameraFix", new CameraFix());
		SmartDashboard.putData("TakePicture", new Snappy());
		SmartDashboard.putData("ChangeDashBoardLayout", new DashboardToggle());
	}
	
	private JoystickButton JoystickButton(Joystick driveStick2, int i) {
		// TODO Auto-generated method stub
		return null;
	}
	
	
}
