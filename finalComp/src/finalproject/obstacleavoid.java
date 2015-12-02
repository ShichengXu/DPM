package finalproject;

import finalproject.Navigation.State;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.port.*;

public class obstacleavoid extends Thread {
	private static EV3LargeRegulatedMotor leftMotor, rightMotor,sensorMotor;
	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider touch1,touch2;
	private float[] touchSample1,touchSample2;

	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 100;

	
	Navigation nav;
	boolean safe;
	private int bandCenter=20;
	private int motorStraight = 200, FILTER_OUT = 20;
	private double distance;
	private int filterControl;
	private double proportion=20;	
	private double perror;
	private double amperror;
	private double angleSave;
	private double distLeft, distRight,distMid;
	private int select=4;
    private static final int maxSpeed=400;

	public obstacleavoid(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor,EV3LargeRegulatedMotor sensorMotor,Odometer odometer,SampleProvider usSensor, float[] usData,SampleProvider touch1, SampleProvider touch2,
			float[] touchSample1,float[] touchSample2) 
	{
		this.leftMotor=leftMotor;
		this.rightMotor=rightMotor;
		this.sensorMotor=sensorMotor;
		this.odometer=odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		this.touchSample1=touchSample1;
		this.touchSample2=touchSample2;
		this.touch1=touch1;
		this.touch2=touch2;
		safe = false;

	}


	public void run() { 
		leftMotor.setAcceleration(2000);
		rightMotor.setAcceleration(2000);
		
		
		distMid=getFilteredData();
		sensorMotor.rotate(45);
		Delay.msDelay(500);
		distLeft=getFilteredData();
		sensorMotor.rotate(-90);
		Delay.msDelay(500);
		distRight=getFilteredData();
		sensorMotor.rotate(45);
		

		if(distLeft>distMid && distLeft>distRight)
		{
			select = 0;
		}
		else if(distRight>=distMid && distRight>=distLeft)
		{
			select = 2;    //turn right
		}
		
	 if(select==0)
    {
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		sensorMotor.setAcceleration(1000);
		angleSave=odometer.getAng();
		leftMotor.rotate(-convertAngle(2.2, 15, 89.8), true);
		rightMotor.rotate(convertAngle(2.2, 15, 89.8), false);
		sensorMotor.rotate(-85);
		
   
		while(safe==false||collide()==false)
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
		if(distance>60){
			// if distance >80, the robot is about to do a U-turn
			leftMotor.setSpeed(maxSpeed); 
			rightMotor.setSpeed(motorStraight); 
			//right wheel will speed up in order to turn left
			leftMotor.forward();
			rightMotor.forward();
		}
	/*	else if(distance<8){
			// if distance<8, the robot is going to hit the wall
			leftMotor.setSpeed(motorStraight+50);
			rightMotor.setSpeed(motorStraight+50);
			leftMotor.forward();
			rightMotor.backward();
			// right motor rolls backwards to respond.
		}
		*/
		else 
		{
			//general proportion error correction
			rightMotor.setSpeed(motorStraight);
			if((motorStraight+amperror)>maxSpeed)
			{
				leftMotor.setSpeed(maxSpeed);
			}
			//left wheel remains constant speed
			else{
			leftMotor.setSpeed((int) (motorStraight+amperror));
			}
			//right wheel will continuously correcting its speed according to errors. 
			leftMotor.forward();
			rightMotor.forward();
		}
		
		if(CloseTo(rightDegreeHelper(angleSave,odometer.getAng()),-90))
		{
			sensorMotor.rotate(85);
			safe=true;
		}
		
	}
  }
		   
		   
		   
		   
		   if(select ==2)
		   {
			   leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				sensorMotor.setAcceleration(1000);
				angleSave=odometer.getAng();
				leftMotor.rotate(convertAngle(2.2, 15, 89.8), true);
				rightMotor.rotate(-convertAngle(2.2, 15, 89.8), false);
				sensorMotor.rotate(85);
			   
				while(safe==false||collide()==false)
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
				if(distance>60){
					// if distance >80, the robot is about to do a U-turn
					leftMotor.setSpeed(motorStraight); 
					rightMotor.setSpeed(maxSpeed); 
					//right wheel will speed up in order to turn left
					leftMotor.forward();
					rightMotor.forward();
				}
			/*	else if(distance<10){
					// if distance<8, the robot is going to hit the wall
					rightMotor.setSpeed(motorStraight+50);
					rightMotor.setSpeed(motorStraight+50);
					rightMotor.forward();
					leftMotor.backward();
					// right motor rolls backwards to respond.
				}
				*/
				else 
				{
					//general proportion error correction
					leftMotor.setSpeed(motorStraight);
					if((motorStraight+amperror)>maxSpeed)
					{
						rightMotor.setSpeed(maxSpeed);
					}
					//left wheel remains constant speed
					else{
					rightMotor.setSpeed((int) (motorStraight+amperror));
					}
					//right wheel will continuously correcting its speed according to errors. 
					leftMotor.forward();
					rightMotor.forward();
				}
						
				if(CloseTo(leftDegreeHelper(angleSave,odometer.getAng()),90))
				{
					sensorMotor.rotate(-85);
					safe=true;
				}
				
			}
				
			   
		   }
		   if(collide()==true)
		   {
			   leftMotor.stop(true);
			   rightMotor.stop(false);
			   leftMotor.rotate(convertDistance(2.2,-10),true);
   	   		   rightMotor.rotate(convertDistance(2.2,-10),false);
 
		   }
		   
}
	
	public double leftDegreeHelper(double angle1,double angle2)
	{
		if(angle2-angle1<=0)
		{
			return(angle2-angle1+360);
		}
		return angle2-angle1;
	}
	
	public double rightDegreeHelper(double angle1,double angle2)
	{
		if(angle2-angle1>=180)
		{
			return(angle2-angle1-360);
		}
		return angle2-angle1;
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
		if (b - 16 < a && a < b + 16) { // if a is within 2cm away from b return
										// true otherwise return false.
			return true;
		}
		return false;
	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		if (distance > 100) {
			distance = 100;
		}
		return distance;
	}
	
	private boolean collide()
	{
		touch1.fetchSample(touchSample1,0);
		touch2.fetchSample(touchSample2,0);
		if(touchSample1[0]==1||touchSample2[0]==1)
		{
			return true;
		}
		return false;
	}

}