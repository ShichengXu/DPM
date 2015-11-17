package finalproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.*;

public class obstacleavoid extends Thread {
	private static EV3LargeRegulatedMotor leftMotor, rightMotor,sensorMotor;
	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;

	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 100;

	
	Navigation nav;
	boolean safe;
	private int bandCenter=30;
	private int motorStraight = 150, FILTER_OUT = 20;
	private double distance;
	private int filterControl;
	private double proportion=13;	
	private double perror;
	private double amperror;
	private double angleSave;


	public obstacleavoid(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor,EV3LargeRegulatedMotor sensorMotor,Odometer odometer,SampleProvider usSensor, float[] usData) 
	{
		this.leftMotor=leftMotor;
		this.rightMotor=rightMotor;
		this.sensorMotor=sensorMotor;
		this.odometer=odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		safe = false;

	}


	public void run() { 
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		sensorMotor.setAcceleration(1000);
		angleSave=odometer.getAng();
		leftMotor.rotate(convertAngle(2.2, 15, 89.8), true);
		rightMotor.rotate(-convertAngle(2.2, 15, 89.8), false);
		sensorMotor.rotate(60);
		

		while(safe==false)
		{
		if (getFilteredData() >100 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
		} else if (getFilteredData() > 100){
			// set distance to this.distance
			distance=100;
		} else {
			// distance went below 100, therefore reset everything.
			filterControl = 0;
			distance = getFilteredData();
		}

		perror = distance-bandCenter;
		amperror= perror*proportion;
		if(distance>55){
			// if distance >80, the robot is about to do a U-turn
			leftMotor.setSpeed(motorStraight); 
			rightMotor.setSpeed(300); 
			//right wheel will speed up in order to turn left
			leftMotor.forward();
			rightMotor.forward();
		}
		else if(distance<10){
			// if distance<8, the robot is going to hit the wall
			leftMotor.setSpeed(motorStraight+50);
			rightMotor.setSpeed(motorStraight+50);
			leftMotor.forward();
			rightMotor.backward();
			// right motor rolls backwards to respond.
		}
		else 
		{
			//general proportion error correction
			leftMotor.setSpeed(motorStraight);
			if((motorStraight+amperror)>300)
			{
				rightMotor.setSpeed(300);
			}
			//left wheel remains constant speed
			rightMotor.setSpeed((int) (motorStraight+amperror));
			//right wheel will continuously correcting its speed according to errors. 
			leftMotor.forward();
			rightMotor.forward();
		}
		
		if(CloseTo(odometer.getAng(),angleSave))
		{
			sensorMotor.rotate(-60);
			safe=true;
		}
		
	}

	}
	
	
	public boolean resolved() {
		return safe;
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	private static boolean CloseTo(double a, double b) {
		if (b - 1 < a && a < b + 1) { // if a is within 2cm away from b return
										// true otherwise return false.
			return true;
		}
		return false;
	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		if (distance > 60) {
			distance = 60;
		}

		return distance;
	}

}