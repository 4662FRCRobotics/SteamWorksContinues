package org.usfirst.frc.team4662.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	
	//CAN
	public static int leftMotor1;
	public static int leftMotor2;
	public static int rightMotor1;
	public static int rightMotor2;
	
	//PWM
	public static int ropeMotor1;
	public static int ropeMotor2;
	
	public static int shooterMotor1;
	public static int gateServo;
	
	public static int loaderMotor;
		//not in init yet 
	public static boolean bDashboardComp;
	public static String fileIniName;
	
	private static void writeIni( String fileIni, String iniText ) {
        BufferedWriter output = null;
        try {
            File file = new File( fileIni );
            output = new BufferedWriter(new FileWriter(file));
            output.write(iniText);
        } catch ( IOException e ) {
            e.printStackTrace();
        } finally {
          if ( output != null ) {
            try {
				output.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
          }
        }
	}
	
	private static String readFile( String file ) throws IOException {
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
	
	public static void init(){
		fileIniName = "/home/lvuser/Robot.ini";
		String strRobotINI;
        
        try {
        	strRobotINI = readFile(fileIniName);
        	
        } catch (IOException e) {
        	strRobotINI = "comp";
        }
        if (strRobotINI == "comp") {
        	 bDashboardComp = true;
        } else {
        	 bDashboardComp = false;
        }
        
        //CAN
		leftMotor1 = 4;
		leftMotor2 = 5;
		rightMotor1 = 2;
		rightMotor2 = 3;
		
		//PWM
		ropeMotor1 = 0;
		ropeMotor2 = 1;
		
		shooterMotor1 = 2;
			//previously boilerLoaderMotor1
		gateServo = 3;
		
		loaderMotor = 4;
	}
	public static void DashboardToggle() {
		bDashboardComp = !bDashboardComp;
	}
	
	public static boolean isDashboardComp() {
		return bDashboardComp;
	}
	
	public static void writeIni() {
		String strRobotIni;
		if ( bDashboardComp ) {
			strRobotIni = "comp";
		} else {
			strRobotIni = "test";
		}
		
		writeIni(fileIniName, strRobotIni);
	}
	
}
