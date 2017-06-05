                              package org.usfirst.frc.team4662.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.StringReader;

import org.usfirst.frc.team4662.robot.Robot;
/**
 *
 */
public class Autonomous extends CommandGroup {
	private String readFile( String file ) throws IOException {
		BufferedReader reader = new BufferedReader( new FileReader (file));
		String	       line = null;
		StringBuilder   stringBuilder = new StringBuilder();
		boolean bFirstTime = true;
		while( ( line = reader.readLine())!= null){
			line = line.trim();
			if (bFirstTime == true) {
				bFirstTime = false;
			} else {
				stringBuilder.append( "|" );
			}
			stringBuilder.append( line );
		}
		reader.close();
		return stringBuilder.toString();
	}
    public Autonomous() {
    	requires (Robot.driveSystem);
    	double dDistance = 0;
    	double dSpeed = 0.6;
    	double dWaitDuration = 2;
    	double dAngle = 0;
    	double dTimeOutVal = 1.5;
    	double dMomentumThreshold = 120; 
    	
    	//Preferences prefs = Preferences.getInstance();
    	SmartDashboard.putString("AutoCommandGroup", "Start");
    	
        	String strFileName = "/home/lvuser/Autonomous/SteamDefault.txt";
        	
        	String autoCommandList ="";
			try {
				autoCommandList = readFile(strFileName);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
        	String[] autoArray = autoCommandList.split("\\|");
	    	
	    	//SmartDashboard.putNumber ("NumNodes", iNumNodes);
	    	//for(i = 0; i < iNumNodes; i++) {
	    	for(int i =0; i < autoArray.length; i = i + 2){
	    		//node = nlAutoAttacks.item(i);
	    		//String strAutoCommand = node.getNodeName();
	    		String strAutoCommand = autoArray[i];
	    		//SmartDashboard.putString("Direction", strAutoCommand);
	    		switch (strAutoCommand) {
	    			case "throttle":
	    				dSpeed = Double.valueOf(autoArray[i + 1]);
	    				SmartDashboard.putNumber("Speed", dSpeed);
	    				break;
	    			case "forward": 
	    	    		dDistance = Double.valueOf(autoArray[i + 1]);
	    	    		//SmartDashboard.putNumber("FDistance", dDistance);
	    				addSequential (new DriveDistancePID(dDistance, dSpeed));
	    				if (dDistance * dSpeed >= dMomentumThreshold) {
	    					addSequential (new WaitForIt(dWaitDuration));
	    				}
	    				break;
	    			case "reverse": 
	    	    		dDistance = Double.valueOf(autoArray[i + 1]);
	    	    		//SmartDashboard.putNumber("RDistance", dDistance);
	    				addSequential (new DriveDistancePID(-dDistance, dSpeed));
	    				if (dDistance >= 100) {
	    					addSequential (new WaitForIt(dWaitDuration));
	    				}
	    				break;
	    				
	    			case "curveleft": 
	    				dTimeOutVal = Double.valueOf(autoArray[i + 1]);
	    				addSequential (new DriveCurve(-45, dSpeed, dTimeOutVal));	    		
	    				break;
	    				
	    			case "curveright": 
	    				dTimeOutVal = Double.valueOf(autoArray[i + 1]);
	    				addSequential (new DriveCurve(45, dSpeed, dTimeOutVal));	    		
	    				break;
	    				
	    			case "rotate":
	    	    		dAngle = Integer.valueOf(autoArray[i + 1]);
	    	    		//SmartDashboard.putNumber("RAngle", dDistance);
	    				addSequential (new GyroRotatePID(dAngle, dSpeed));
	    				break;
	    				
	    			case "gearforward":
	    				addSequential (new GearFront());
	    				break;
	    				
	    			case "shooterforward":
	    				addSequential (new ShooterFront());
	    				break;
	    			/*case "locatetarget":
	    	    		iDistance = Integer.valueOf(autoArray[i + 1]);
	    	    		//SmartDashboard.putNumber("Distance", iDistance);
	    				addSequential (new LocateTarget());
	    				break;
	    			*/
	    			case "shoot":
	    				dTimeOutVal = Double.valueOf(autoArray[i + 1]);
	    				addSequential (new BallShoot(dTimeOutVal));
	    				break;
	    			case "findandshoot":
	    				dTimeOutVal = Double.valueOf(autoArray[i + 1]);
	    				addSequential (new FindBoiler(dSpeed));
	    				//addSequential (new BallShoot(dTimeOutVal));
	    				break;
	    			case "wait":
	    				dWaitDuration = Double.valueOf(autoArray [i + 1]);
	    				addSequential (new WaitForIt(dWaitDuration));
	    				break;
	    				
	    				
	    					    			
	    			default:
	    				//SmartDashboard.putString("HI", "Default");
	    				
	    		}
	    		//SmartDashboard.putString("AutoCommandGroup", "looping" + i);
	    	}
    	
    }
}
