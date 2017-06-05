package org.usfirst.frc.team4662.robot.subsystems;

import org.usfirst.frc.team4662.robot.Robot;
import org.usfirst.frc.team4662.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class VisionSystem extends Subsystem {
	private NetworkTable VisionTable;
	private boolean m_bGearForwardDrive;
	private boolean m_bGearForwardVision;
	private boolean m_bIsVisionOn;
	
	private double m_dR1PixelX;
	private double m_dR1PixelY;
	private double m_dR1PixelW;
	private double m_dR1PixelH;
	private boolean m_bIsTargetFound;
	private final double BOILER_VERTICAL_MIDPOINT = 83;
	private final double BOILER_SHOOT_SWEETSPOT_DISTANCE = 72;
	private final double BOILER_CAMERA_VERTICAL_OFFSET = 11.5;
	private final double BOILER_CAMERA_VERTICAL_ANGLE = 36;
	private final double BOILER_CAMERA_HORIZONTAL_OFFSET = 0;
	private final double BOILER_CAMERA_HORIZONTAL_ANGLE = 0;
	private final double BOILER_CAMERA_FOV_X = 50.2;
	private final double BOILER_CAMERA_RES_X = 640;
	private final double BOILER_CAMERA_FOV_Y = 38.9;
	private final double BOILER_CAMERA_RES_Y = 480;
	
	public VisionSystem() {
		VisionTable = NetworkTable.getTable("Vision");
		m_bGearForwardDrive = true;
		m_bGearForwardVision = true;
		m_bIsVisionOn = false;
		
		m_dR1PixelX = -2;
		m_dR1PixelY = -2;
		m_dR1PixelW = -2;
		m_dR1PixelH = -2;
		m_bIsTargetFound = false;
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
 
    public void toggleEnds(){
    	m_bGearForwardDrive = !m_bGearForwardDrive;
       	m_bGearForwardVision = !m_bGearForwardVision;
    	VisionTable.putBoolean("IsGearDrive", m_bGearForwardVision);
    }
    
    public void cameraFix() {
    	m_bGearForwardDrive = !m_bGearForwardDrive;
    	VisionTable.putBoolean("IsGearDrive", m_bGearForwardVision);
    }
    
    public void GearCam(){
    	m_bGearForwardDrive = true;
    	m_bGearForwardVision = true;
    	VisionTable.putBoolean("IsGearDrive", m_bGearForwardVision);
    }
    
    public void ShooterCam(){
    	m_bGearForwardDrive = false;
    	m_bGearForwardVision = false;
    	VisionTable.putBoolean("IsGearDrive", m_bGearForwardVision);
    }
    
    public void LightToggle() {
    	m_bIsVisionOn = !m_bIsVisionOn;
    	SmartDashboard.putBoolean("IsVisionOn", m_bIsVisionOn);
    	VisionTable.putBoolean("IsVisionOn", m_bIsVisionOn);
    }
    public void LightOn() {
    	m_bIsVisionOn = true;
    	SmartDashboard.putBoolean("IsVisionOn", m_bIsVisionOn);
    	VisionTable.putBoolean("IsVisionOn", m_bIsVisionOn);
    }
    public void LightOff() {
    	m_bIsVisionOn = false;
    	SmartDashboard.putBoolean("IsVisionOn", m_bIsVisionOn);
    	VisionTable.putBoolean("IsVisionOn", m_bIsVisionOn);
    }
    public void VisionNTDisplay() {
    	m_bIsTargetFound = VisionTable.getBoolean("IsTargetFound", m_bIsTargetFound);
    	m_dR1PixelX = VisionTable.getNumber("Target1X", m_dR1PixelX);
        m_dR1PixelY = VisionTable.getNumber("Target1Y", m_dR1PixelY);
    	m_dR1PixelW = VisionTable.getNumber("Target1Width", m_dR1PixelW);
    	m_dR1PixelH = VisionTable.getNumber("Target1Height", m_dR1PixelH);
    	    	
    	SmartDashboard.putBoolean("IsTargetFound", m_bIsTargetFound);
    	if ( !RobotMap.isDashboardComp()) {
	    	SmartDashboard.putNumber("Target1X", m_dR1PixelX);
	    	SmartDashboard.putNumber("Target1Y", m_dR1PixelY);
	    	SmartDashboard.putNumber("Target1Width", m_dR1PixelW);
	    	SmartDashboard.putNumber("Target1Height", m_dR1PixelH);
    	}
    	SmartDashboard.putNumber("TargetTurnAngle", getBoilerAngle());
    	SmartDashboard.putNumber("TargetDriveDistance", getBoilerDistance());
    }
    
    public void takePic() {
    	VisionTable.putBoolean("Take Pic", true);
    }
    
    public boolean isVisionOn() {
    	return m_bIsVisionOn;
    }
    
    public boolean isGearForwardDrive() {
    	return m_bGearForwardDrive;
    }
    
    public boolean isGearForwardVision() {
    	return m_bGearForwardVision;
    }
    
    public boolean isTargetFound() {
    	return VisionTable.getBoolean("IsTargetFound", m_bIsTargetFound);
    }
    
    public double getBoilerAngle() {
    	double midpoint = m_dR1PixelX + (m_dR1PixelW/2);
    	double midpointRatio = (midpoint - (BOILER_CAMERA_RES_X / 2)) / (BOILER_CAMERA_RES_X / 2);
    	double cameraAngle = midpointRatio * (BOILER_CAMERA_FOV_X / 2); 
    	return cameraAngle + BOILER_CAMERA_HORIZONTAL_ANGLE ;
    }
    
    public double getBoilerDistance() {
    	double midpoint = m_dR1PixelY + (m_dR1PixelH/2);
    	double midpointRatio = (midpoint - (BOILER_CAMERA_RES_Y / 2)) / (BOILER_CAMERA_RES_Y / 2);
    	double cameraAngle = -1 * (midpointRatio * (BOILER_CAMERA_FOV_Y / 2)); 
    	double targetAngle = cameraAngle + (BOILER_CAMERA_VERTICAL_ANGLE);
    	double targetTangent = Math.tan(Math.toRadians(targetAngle));
    	double targetDistance = (BOILER_VERTICAL_MIDPOINT - BOILER_CAMERA_VERTICAL_OFFSET) / targetTangent;
    	return targetDistance - BOILER_SHOOT_SWEETSPOT_DISTANCE;
    }
    
    public double getGearAngle() {
    	double midpoint = m_dR1PixelX + (m_dR1PixelW/2);
    	double midpointRatio = (midpoint - (BOILER_CAMERA_RES_X / 2)) / (BOILER_CAMERA_RES_X / 2);
    	double cameraAngle = midpointRatio * (BOILER_CAMERA_FOV_X / 2); 
    	return cameraAngle + BOILER_CAMERA_HORIZONTAL_ANGLE ;
    }
}