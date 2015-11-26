package finalproject;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class searching {
	private static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	private static final EV3MediumRegulatedMotor claw = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private static EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;
	Navigation nav;
	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider touch1, touch2;
	private float[] touchSample1, touchSample2;
	private double distance;
	private int filterControl=0;
	private static int colorfilterControl=0;
	private double proportion = 5;
	private double perror;
	private double amperror;
	private int cruisingSpeed = 100, FILTER_OUT = 100, motorStraight = 200;
    private static int colorTemp=0,colourFilter=100;
	private static final int maxSpeed = 300;
	private int bandCenter = 12;
	static boolean found;
	private double temp = 100, targetAng;

	public searching(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor sensorMotor, Odometer odometer, SampleProvider usSensor, float[] usData,
			SampleProvider touch1, SampleProvider touch2, float[] touchSample1, float[] touchSample2, Navigation nav) {
		this.nav = nav;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		this.touchSample1 = touchSample1;
		this.touchSample2 = touchSample2;
		this.touch1 = touch1;
		this.touch2 = touch2;
		found = false;
	}

	public void doSearching() {
		
		synchronized(this){			  
			leftMotor.setSpeed(cruisingSpeed);
			rightMotor.setSpeed(cruisingSpeed);	
			 }
	nav.turnTo(90);
	//	nav.turnTo(0);
//		while (odometer.getAng() >3) {
//			synchronized(this){			  
//				leftMotor.forward();
//				rightMotor.backward();
//				 }
//		
//			temp = getFilteredData();
//			if (getFilteredData() < temp) {
//				targetAng = odometer.getAng();
//			}
//
//		}
//		nav.turnTo(targetAng);
//
//		while (getFilteredData() > 7) {
//			synchronized(this){		
//				leftMotor.setSpeed(300);
//				rightMotor.setSpeed(300);	
//				leftMotor.forward();
//				rightMotor.forward();
//				 }
//			
//		}
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//	    nav.turnTo(80);

		while (colourFilter() != 1) {
			synchronized(this){		
				leftMotor.forward();
				rightMotor.forward();
				 }
		}
		Sound.beep();
		Sound.beep();
		leftMotor.stop(true);
		rightMotor.stop(false);
		synchronized(this){	
		leftMotor.rotate(convertDistance(2.2,-10),true);
		rightMotor.rotate(convertDistance(2.2,-10),false);
		}
		synchronized(this){	
			claw.rotate(270);

rightMotor.rotate(convertDistance(2.2, 24));
		}
		synchronized(this){	
//claw.rotate(270);
leftMotor.rotate(convertDistance(2.2,-15),true);
rightMotor.rotate(convertDistance(2.2,-15),false);
		claw.rotate(-200);
		claw.stop();
		}
	}

	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		if (distance > 60) {
			distance = 60;
		}
		return distance;
	}
	
	public synchronized void setSpeed(int leftSpeed, int rightSpeed)
	{
			leftMotor.setSpeed(leftSpeed);
			rightMotor.setSpeed(rightSpeed);
			leftMotor.forward();
			rightMotor.forward();
	}
	
/*
	private boolean collide() {
		touch1.fetchSample(touchSample1, 0);
		touch2.fetchSample(touchSample2, 0);
		if (touchSample1[0] == 1 || touchSample2[0] == 1) {
			return true;
		}
		return false;
	}
*/
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
	private static int colourFilter()
	{
		while(true)
		{
	//	colorTemp=colour();
		if (colour()==colorTemp && colorfilterControl<colourFilter) {
			// bad value, do not set the distance var, however do increment
			// the filter value
			colorfilterControl++;
		}
		else if(colour()!=colorTemp)
		{
			colorfilterControl = 0;
			colorTemp=colour();
		}
		else 
		{
			colorfilterControl = 0;
			return colour();
			}
		}
		
	}
		
		
	private static int colour() {
		float[] sampleRGB = new float[3];
		colorSensor.getRGBMode().fetchSample(sampleRGB, 0);

		if ((sampleRGB[0] * 1000 < sampleRGB[1] * 1000) && (sampleRGB[1] * 1000 < sampleRGB[2] * 1000)) {
			return 1; // dark blue
		}
		else if ((sampleRGB[0] * 1000 < sampleRGB[1] * 1000) && (sampleRGB[1] * 1000 > sampleRGB[2] * 1000)
				&& (((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) < 0.8)) {
			return 2; // light blue
		}
		else if ((sampleRGB[0] * 1000 > sampleRGB[1] * 1000)
				&& ((int) ((sampleRGB[1] * 1000) / (sampleRGB[2] * 1000)) > 2)) {
			return 3; // yellow
		}
		else if ((sampleRGB[0] * 1000 > sampleRGB[1] * 1000)
				&& ((int) ((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) > 1)) {
			return 4; // red
		}
		else if ((Math.round((sampleRGB[0] * 1000) * 0.7 / (sampleRGB[2] * 1000)) == 1)
				&& (((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) < 1.1)
				&& (((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) > 0.9)) {
			return 5; // white
		}
		else {return 0;}
	}

	public static boolean isfound() {
		return found;
	}

}