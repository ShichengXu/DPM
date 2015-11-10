package finalproject;

//import finalproject.LCDInfo;
//import lab5obj.Odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.Button;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3TouchSensor;

public class FinalProject {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	// Touch sensor port connected to input S3
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
	
	
	public static void main(String[] args) {
	
		Odometer odo = new Odometer(leftMotor, rightMotor,30,true);
		SampleProvider usValue = usSensor.getMode("Distance");
		
		Localization usl = new Localization(odo, usValue, usData,touch1,touch2,touchSample1,touchSample2,
				Localization.LocalizationType.FALLING_EDGE, leftMotor,
				rightMotor);
		usl.doLocalization();
		
		/*
		while(true)
		{
			touch1.fetchSample(touchSample1,0);
			touch2.fetchSample(touchSample2, 0);
			LCD.drawString("value "+touchSample1[0],0,1);
			LCD.drawString("value2 "+touchSample2[0],0,2);
			
			
		}
		*/
	}
	
}