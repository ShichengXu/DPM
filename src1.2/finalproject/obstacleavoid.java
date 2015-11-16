package finalproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.*;

public class obstacleavoid extends Thread {
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;

	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 100;
	private static double theta = 0, deltaTheta = 0;
	private static double currentX, currentY, currentT, goToX, goToY,
			saveTheta;
	private static double LEFTR = 2.1, RIGHTR = 2.1, WIDTH = 15;
	private static boolean isNavigating = true, good = true;
	private static boolean isTraveling = true, isLeftTraveling = true,
			isRightTraveling = true;
	private static double counter, error, BANDCENTER = 32;
	private SampleProvider touch1,touch2;
	private float[] touchSample1,touchSample2;
	Navigation nav;
	boolean safe;

	public obstacleavoid(Navigation nav) 
	{
		this.nav = nav;
		safe = false;

	}


	public void run() { 
		
// do something
		safe=true;

	}
	
	
	public boolean resolved() {
		return safe;
	}

}