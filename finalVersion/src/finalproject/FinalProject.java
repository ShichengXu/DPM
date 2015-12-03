package finalproject;

import java.io.IOException;

import finalproject.StartCorner;
import finalproject.Transmission;
import finalproject.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.Button;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3TouchSensor;

/**
 * 2015 DPM final project. main class
 * 
 * @author Ryan
 */
public class FinalProject {
	// private static final String SERVER_IP = "192.168.10.40";
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
	private static final Port leftTouchPort = LocalEV3.get().getPort("S3");
	private static final Port rightTouchPort = LocalEV3.get().getPort("S4");
	static SensorModes usSensor = new EV3UltrasonicSensor(usPort);

	static SampleProvider usValue = usSensor.getMode("Distance");
	static SampleProvider touch1 = new EV3TouchSensor(leftTouchPort);
	static SampleProvider touch2 = new EV3TouchSensor(rightTouchPort);

	static float[] touchSample1 = new float[touch1.sampleSize()];
	static float[] touchSample2 = new float[touch2.sampleSize()];
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
	public double x, y;
	public static double[] targetXYT;
	public static double[] targetOZxy1, targetOZxy2;
	public static double[] homeZoneBL, homeZoneTR, opponentHomeZoneBL,
			opponentHomeZoneTR, dropZone;

	public static void main(String[] args) {
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		OdometerDisplay odometryDisplay = new OdometerDisplay(odo);
		Navigation nav = new Navigation(leftMotor, rightMotor, odo, usValue,
				usData, touch1, touch2, touchSample1, touchSample2, sensorMotor);
		SampleProvider usValue = usSensor.getMode("Distance");

		Localization usl = new Localization(odo, usValue, usData, touch1,
				touch2, touchSample1, touchSample2,
				Localization.LocalizationType.FALLING_EDGE, leftMotor,
				rightMotor, sensorMotor);
		wallfollowing wf = new wallfollowing(leftMotor, rightMotor, odo,
				usSensor, usData, touch1, touch2, touchSample1, touchSample2,
				sensorMotor);

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

			startingCorner = corner.getId();
			// print out the transmission information
			conn.printTransmission();
		}

		// perform the localization
		usl.doLocalization();
		targetXYT = findTargetOZxy(startingCorner);
		if (startingCorner == 1) {
			targetCorner = 3;
		} else if (startingCorner == 2) {
			targetCorner = 4;
		} else if (startingCorner == 3) {
			targetCorner = 1;
		} else {
			targetCorner = 2;
		}
        //perform wall follower to get to nearest corner
		if (startingCorner == 1) {

			wf.doWallfollowing(1, 300, 0);
			wf.doWallfollowing(2, 300, 300);
			nav.TravelTo(targetXYT[0] * 30, targetXYT[1] * 30);
			nav.doNavigation(true);

		} else if (startingCorner == 2) {

			wf.doWallfollowing(2, 300, 300);

			nav.TravelTo(targetXYT[0] * 30, targetXYT[1] * 30);
			nav.doNavigation(true);

		} else if (startingCorner == 3) {

			wf.doWallfollowing(2, 300, 0);

			nav.TravelTo(targetXYT[0] * 30, targetXYT[1] * 30);
			nav.doNavigation(true);

		} else if (startingCorner == 4) {

			if (targetCorner == 2) {
				wf.doWallfollowing(4, 0, 0);
				nav.TravelTo(targetXYT[0] * 30, targetXYT[1] * 30);
				nav.doNavigation(true);
			}

		}
	
		nav.turnTo(targetXYT[2]);

		searching search = new searching(leftMotor, rightMotor, sensorMotor,
				odo, usSensor, usData, touch1, touch2, touchSample1,
				touchSample2, nav);
		// searching:

		search.doSearching();

		// retrieval:
		retrieval();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

	/**
	 * find which corner is the opponentHomeZone closest to.
	 * 
	 * @return corner ID -- 1,2,3 or 4.
	 */
	public static int closestCorner() {
		midPointX = (opponentHomeZoneBL_X + opponentHomeZoneTR_X) / 2;
		midPointY = (opponentHomeZoneBL_Y + opponentHomeZoneTR_Y) / 2;
		if (midPointX < 5 && midPointY < 5) {
			return 1;
		} else if (midPointX >= 5 && midPointY < 5) {
			return 2;
		} else if (midPointX >= 5 && midPointY >= 5) {
			return 3;
		} else {
			return 4;
		}
	}

	/**
	 * calculate which corner in opponent zone is the closest to the corner
	 * 
	 * @param startCorner
	 *            start corner as a reference to translate all coordinates.
	 * @return x and y coordinates in an array.
	 */

	private static double[] findTargetOZxy(int atCorner) {
		double[] result = new double[3];

		if (atCorner == 1) {
			result[0] = opponentHomeZoneTR_X;
			result[1] = opponentHomeZoneBL_Y;
			result[2] = 180;
			// result[0]=opponentHomeZoneBL_X;
			// result[1]=opponentHomeZoneBL_Y;
			// result[2]=90;
		} else if (atCorner == 2) {
			result[0] = opponentHomeZoneTR_X;
			result[1] = opponentHomeZoneTR_Y;
			result[2] = 270;
			// result[0]=opponentHomeZoneTR_X;
			// result[1]=opponentHomeZoneBL_Y;
			// result[2]=180;
		} else if (atCorner == 3) {
			result[0] = opponentHomeZoneBL_X;
			result[1] = opponentHomeZoneTR_Y;
			// result[0]=opponentHomeZoneTR_X;
			// result[1]=opponentHomeZoneTR_Y;
			result[2] = 0;
		} else {
			result[0] = opponentHomeZoneBL_X;
			result[1] = opponentHomeZoneBL_Y;
			// result[0]=opponentHomeZoneBL_X;
			// result[1]=opponentHomeZoneTR_Y;
			result[2] = 90;
		}

		return result;
	}

	/**
	 * travel to dropZone.
	 */
	private static void retrieval() {
		// nav.TravelTo(20,20);

		nav.TravelTo(dropZone[0], dropZone[1]);
		nav.doNavigation(true);
	}

}