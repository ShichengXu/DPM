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

public static  int 	homeZoneBL_X;
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
public int temp;
	
	
	
	
	public static void main(String[] args) {
		/*
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
		
			// print out the transmission information
			conn.printTransmission();
		}
		// stall until user decides to end program
		Button.ESCAPE.waitForPress();
		
		*/
		leftMotor.setAcceleration(1200);
		rightMotor.setAcceleration(1200);
		Odometer odo = new Odometer(leftMotor, rightMotor,30,true);
		OdometerDisplay odometryDisplay = new OdometerDisplay(odo);
		Navigation nav = new Navigation (leftMotor,rightMotor,odo,usValue,usData,touch1,touch2,touchSample1,touchSample2,sensorMotor);
		SampleProvider usValue = usSensor.getMode("Distance");
		
		Localization usl = new Localization(odo, usValue, usData,touch1,touch2,touchSample1,touchSample2,
				Localization.LocalizationType.FALLING_EDGE, leftMotor,
				rightMotor);
	//	usl.doLocalization();
	//	nav.TravelTo(20,20);
	//	nav.TravelTo(opponentHomeZoneBL_X*30,opponentHomeZoneBL_Y*30);
	//	nav.doNavigation(false);	
	//	completeCourse();
		
		searching search = new searching(leftMotor, rightMotor,sensorMotor, odo,usSensor,usData,touch1, touch2,
				   touchSample1,touchSample2,nav);
		search.doSearching();
	//	retrieval();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}


private static void completeCourse() {
	//nav.TravelTo(20,20);
			nav.TravelTo(opponentHomeZoneBL_X*30,opponentHomeZoneBL_Y*30);
			while(nav.isTravelling()){
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
	}
private static void retrieval() {
	//nav.TravelTo(20,20);
			nav.TravelTo(dropZone_X*30,dropZone_Y*30);
			while(nav.isTravelling()){
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}		
	}
private void coordinateHelper(int startCorner, int x, int y)
{
	if(startCorner==1)
	{
		x=x;
		y=y;
	}
	else if (startCorner==2)
	{
		temp=y;
		y=10-x;
		x=temp;
	}
	else if(startCorner==3)
	{
		x=10-x;
		y=10-y;
	}
	else if(startCorner==4)
	{
		temp=y;
		y=x;
		x=10-temp;
	}
	}



}