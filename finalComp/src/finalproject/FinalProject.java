package finalproject;


import java.io.IOException;

import finalproject.StartCorner;
import finalproject.Transmission;
import finalproject.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3TouchSensor;

/**
 * 2015 DPM final project. main class
 * @author Ryan
 */
public class FinalProject {
	private static final String SERVER_IP = "192.168.10.200"; 
	private static final int TEAM_NUMBER = 4;
	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	// Left touch sensor port connected to input S3
	// Right touch sensor port connected to input S4
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(
			LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(
			LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(
			LocalEV3.get().getPort("B"));
	
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S2");
	private static final Port leftTouchPort= LocalEV3.get().getPort("S3");
	private static final Port rightTouchPort=LocalEV3.get().getPort("S4");
	static SensorModes usSensor = new EV3UltrasonicSensor(usPort);
	
	static SampleProvider usValue = usSensor.getMode("Distance");
	static SampleProvider touch1 = new EV3TouchSensor(leftTouchPort);
	static SampleProvider touch2 = new EV3TouchSensor(rightTouchPort);
	
	static float[] touchSample1=new float[touch1.sampleSize()];
	static float[] touchSample2=new float[touch2.sampleSize()];
	static float[] usData = new float[usValue.sampleSize()];
	private static Navigation nav;

public static int startingCorner;
public static int homeZoneBL_X;
public static int homeZoneBL_Y;
public static int homeZoneTR_X;
public static int homeZoneTR_Y;
public static int opponentHomeZoneBL_X;
public static int opponentHomeZoneBL_Y;
public static int opponentHomeZoneTR_X;
public static int opponentHomeZoneTR_Y;
public static int dropZone_X;
public static int dropZone_Y;
public static int flagType;
public static int opponentFlagType;
public static int temp;
public static int targetCorner;
public static double midPointX;
public static double midPointY;
public double x,y;
public static double [] targetXYT;
public static double [] targetOZxy1,targetOZxy2;
public static double [] homeZoneBL,homeZoneTR,opponentHomeZoneBL,opponentHomeZoneTR,dropZone;
	
	
	
	public static void main(String[] args) {
		Odometer odo = new Odometer(leftMotor, rightMotor,30,true);
		OdometerDisplay odometryDisplay = new OdometerDisplay(odo);
		Navigation nav = new Navigation (leftMotor,rightMotor,odo,usValue,usData,touch1,touch2,touchSample1,touchSample2,sensorMotor);
		SampleProvider usValue = usSensor.getMode("Distance");
		
		Localization usl = new Localization(odo, usValue, usData,touch1,touch2,touchSample1,touchSample2,Localization.LocalizationType.FALLING_EDGE, 
				leftMotor,rightMotor,sensorMotor);
		wallfollowing wf=new wallfollowing(leftMotor,
				rightMotor,odo, usSensor, usData, touch1,  touch2,
				touchSample1, touchSample2, sensorMotor);
		searching search = new searching(leftMotor, rightMotor,sensorMotor, odo,usSensor,usData,touch1, touch2,
				   touchSample1,touchSample2,nav);
		
	
		WifiConnection conn = null;
		try {
			conn = new WifiConnection(SERVER_IP, TEAM_NUMBER);
		} catch (IOException e) {
			LCD.drawString("Connection failed", 0, 8);
		}
		
		// example usage of Transmission class
		Transmission t = conn.getTransmission();
		if (t == null) {
			LCD.drawString("Failed to read transmission", 0, 5);
		} else {
			StartCorner corner = t.startingCorner;
			homeZoneBL_X = t.homeZoneBL_X;
			homeZoneBL_Y = t.homeZoneBL_Y;
			 homeZoneTR_X = t.homeZoneTR_X;
		     homeZoneTR_Y = t.homeZoneTR_Y;
			 opponentHomeZoneBL_X = t.opponentHomeZoneBL_X;
			opponentHomeZoneBL_Y = t.opponentHomeZoneBL_Y;
			 opponentHomeZoneTR_X = t.opponentHomeZoneTR_X;
				opponentHomeZoneTR_Y = t.opponentHomeZoneTR_Y;
			
			 dropZone_X = t.dropZone_X;
			dropZone_Y = t.dropZone_Y;
			 flagType = t.flagType;
			opponentFlagType = t.opponentFlagType;
		   
			startingCorner=corner.getId();
			// print out the transmission information
			conn.printTransmission();
		}
		
	
		
		usl.doLocalization();
		targetXYT=findTargetOZxy(startingCorner);
		nav.TravelTo(targetXYT[0]*30,targetXYT[1]*30);
		nav.doNavigation(true);
//		Sound.beep();
		nav.turnTo(targetXYT[2]);
//		Sound.buzz();
	//	nav.TravelTo(opponentHomeZoneBL[0], opponentHomeZoneBL[1]);
	//	nav.TravelTo(50,50);
	//	nav.TravelTo(opponentHomeZoneBL_X*30,opponentHomeZoneBL_Y*30);
	//	nav.doNavigation(false);	
		
	//	searching search = new searching(leftMotor, rightMotor,sensorMotor, odo,usSensor,usData,touch1, touch2,
	//			   touchSample1,touchSample2,nav);
		search.doSearching();
        retrieval();
		
		
		
		/*
		targetCorner=closestCorner();
		
		System.out.println("targetCorner "+targetCorner);
		if(targetCorner==2)
		{
			wf.doWallfollowing(1,300,0);
		}
		else if(targetCorner==3)
		{
			wf.doWallfollowing(1, 300, 0);
			wf.doWallfollowing(2, 300, 300);
		}
		else if(targetCorner==4)
		{
			wf.doWallfollowing(1, 300, 0);
			wf.doWallfollowing(2,300,300);
			wf.doWallfollowing(3, 0, 300);
		}
	
		//targetXY=findTargetOZxy(startingCorner);
		targetXYT=findTargetOZxy(targetCorner);
		nav.TravelTo(targetXYT[0],targetXYT[1]);
		nav.doNavigation(true);
		nav.turnTo(targetXYT[2]);
		
		
		//searching:
		
		search.doSearching();
		
		//	retrieval:
		retrieval();
		*/
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

	
	
	/**
	 * find which corner is the opponentHomeZone closest to.
	 * @return corner ID -- 1,2,3 or 4.
	 */
public static int closestCorner()
{
	midPointX=(opponentHomeZoneBL_X+opponentHomeZoneTR_X)/2;
	midPointY=(opponentHomeZoneBL_Y+opponentHomeZoneTR_Y)/2;
	if(midPointX<5 &&midPointY<5)
	{
		return 1;
	}
	else if(midPointX>=5 &&midPointY<5)
	{
		return 2;
	}
	else if(midPointX>=5 && midPointY>=5)
	{
		return 3;
	}
	else {return 4;}
}
	

/**
 * calculate which corner in opponent zone is the closest to the corner
 * @param startCorner start corner as a reference to translate all coordinates.
 * @return x and y coordinates in an array.
 */

private static double[] findTargetOZxy(int atCorner) {
	double[] result = new double[3];
	
	if(atCorner==1)
	{
		result[0]=opponentHomeZoneTR_X;
		result[1]=opponentHomeZoneBL_Y;
		result[2]=180;
		//result[0]=opponentHomeZoneBL_X;
		//result[1]=opponentHomeZoneBL_Y;
		//result[2]=90;
	}
	else if(atCorner==2)
	{
		result[0]=opponentHomeZoneBL_X;
		result[1]=opponentHomeZoneBL_Y;
		result[2]=90;
		//result[0]=opponentHomeZoneTR_X;
		//result[1]=opponentHomeZoneBL_Y;
		//result[2]=180;
	}
	else if (atCorner==3)
	{
		result[0]=opponentHomeZoneTR_X;
		result[1]=opponentHomeZoneTR_Y;
		result[2]=270;
	}
	else 
	{
		result[0]=opponentHomeZoneBL_X;
		result[1]=opponentHomeZoneTR_Y;
		result[2]=0;
	}
	
	return result;
}

/**
 * travel to dropZone. 
 */
private static void retrieval() {
	//nav.TravelTo(20,20);
	       
			nav.TravelTo(dropZone[0],dropZone[1]);
			nav.doNavigation(true);
	}

/**
 * converts static input coordinates into coordinates seen by starting position. All 
 * return values are in cm.
 * @param startCorner the robot can start from corner 1,2,3 or 4.
 * @param x any input x coordinates seen if the robot starts from corner 1
 * @param y any input y coordinates seen if the robot starts from corner 1
 */


}