
package org.usfirst.frc.team4662.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team4662.robot.commands.*;
import org.usfirst.frc.team4662.robot.subsystems.DriveSystem;
import org.usfirst.frc.team4662.robot.subsystems.LoaderSystem;
import org.usfirst.frc.team4662.robot.subsystems.RopeSystem;
import org.usfirst.frc.team4662.robot.subsystems.ShooterSystem;
import org.usfirst.frc.team4662.robot.subsystems.VisionSystem;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public boolean isDriveToggled = false;
	public static DriveSystem driveSystem;
	public static VisionSystem visionSystem;
	public static RopeSystem ropeSystem;
	public static ShooterSystem boilerLoader;
	public static LoaderSystem loaderSystem;
	public static OI oi;
	
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	Thread visionThread;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		RobotMap.init();
		driveSystem = new DriveSystem();
		visionSystem = new VisionSystem();
		ropeSystem = new RopeSystem();
		boilerLoader = new ShooterSystem();
		loaderSystem = new LoaderSystem();
		oi = new OI();
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		driveSystem.dashboardDisplay();
		driveSystem.dashboardGyroDisplay();
		
		boilerLoader.servoClose();

	visionThread = new Thread(() -> {

		//UsbCamera shooterCam = CameraServer.getInstance().startAutomaticCapture(0);
		//UsbCamera gearCam = CameraServer.getInstance().startAutomaticCapture(1);
		UsbCamera shooterCam = new UsbCamera ( "USB Camera 0", 0);
		UsbCamera gearCam = new UsbCamera ( "USB Camera 1", 1);
		
		shooterCam.setResolution(320, 240);
		shooterCam.setFPS(15);
		gearCam.setResolution(320, 240);
		gearCam.setFPS(15);
		
		//CvSinks capture Mats from the camera.
		CvSink cvSinkShooter = CameraServer.getInstance().getVideo(shooterCam);
		CvSink cvSinkGear = CameraServer.getInstance().getVideo(gearCam);
			//CvSourc sends images back to the Dashboard
		CvSource outputStream = CameraServer.getInstance().putVideo("DriveCam", 320, 240);
	
		//Mats are very memory expensive. Reuse this Mat.
		Mat mat = new Mat();
		
		
		//Cannot be "true" bc program will not exist. 
		//Lets robot stop thread when restarting robot code or deploying
		
		while (!Thread.interrupted()) {
			//if toggleDrive
			//unfinished
			//toggleDrive is a Joystick button
			if (!visionSystem.isGearForwardDrive()) {
				cvSinkGear.setEnabled(false);
				cvSinkShooter.setEnabled(true);
				if(cvSinkShooter.grabFrame(mat) == 0) {
					// Send the output the error
					outputStream.notifyError(cvSinkShooter.getError());
					continue;
				}
			} else {
				cvSinkShooter.setEnabled(false);
				cvSinkGear.setEnabled(true);
				if(cvSinkGear.grabFrame(mat) == 0) {
					// Send the output the error
					outputStream.notifyError(cvSinkGear.getError());
					continue;
				}
			}
			// Put a rectangle on the image
			//Imgproc.rectangle(mat, new Point(60, 40), new Point(260, 200),
					//new Scalar(255, 255, 255), 5);
			outputStream.putFrame(mat);
		}
	});
	visionThread.setDaemon(true);
	visionThread.start();
}
		
		
	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = new Autonomous();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		driveSystem.initTeleop();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	
}
