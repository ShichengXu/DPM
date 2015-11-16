package finalproject;


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
	
	
	public static void main(String[] args) {
	//	final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odo = new Odometer(leftMotor, rightMotor,30,true);
		OdometerDisplay odometryDisplay = new OdometerDisplay(odo);
		nav = new Navigation (leftMotor,rightMotor,odo,usValue,usData,touch1,touch2,touchSample1,touchSample2);
		SampleProvider usValue = usSensor.getMode("Distance");
		
		Localization usl = new Localization(odo, usValue, usData,touch1,touch2,touchSample1,touchSample2,
				Localization.LocalizationType.FALLING_EDGE, leftMotor,
				rightMotor);
	//	usl.doLocalization();
		nav.start();	
		
		completeCourse();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
private static void completeCourse() {
		
		int[][] waypoints = {{60,30},{30,30},{30,60},{60,0}};
		
		for(int[] point : waypoints){
			nav.TravelTo(point[0],point[1]);
			while(nav.isTravelling()){
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}

	
	
}