package finalproject;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigation extends Thread {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor sensorMotor;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider touch1,touch2;
	private float[] touchSample1,touchSample2;
	private Odometer odometer;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 100;
	final static int FAST = 300, SLOW = 100, ACCELERATION = 4000;
	private static double theta = 0;
	private static double destx, desty;
	private static double currentX, currentY, currentT, goToX, goToY;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	private static double LEFTR = 2.1, RIGHTR = 2.1, WIDTH = 15;
	private static boolean isNavigating = true;
	enum State {INIT, TURNING, TRAVELLING, EMERGENCY};
	State state;
	private boolean jumpout;

	public Navigation(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, Odometer odometer,SampleProvider usSensor, float[] usData,SampleProvider touch1, SampleProvider touch2,
			float[] touchSample1,float[] touchSample2, EV3LargeRegulatedMotor sensorMotor) {	
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		this.touchSample1=touchSample1;
		this.touchSample2=touchSample2;
		this.touch1=touch1;
		this.touch2=touch2;
		this.sensorMotor=sensorMotor;
//		jumpout=false;
	}



	public void TravelTo(double x, double y) {
		destx=x;
		desty=y;
		isNavigating=true;
		
		
	//	double minAng;
	//	while (!checkIfDone(x,y)) {
	//		minAng = getDestAngle(x,y);
	//		this.turnTo(minAng);
	//		this.setSpeeds(FAST, FAST);
	//	}
	//	this.setSpeeds(0, 0);
	}


	public void turnTo(double angle) {
		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else {
				this.setSpeeds(-SLOW, SLOW);
			}
		}
	}


	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}
	
	protected double getDestAngle(double x, double y) {
		double minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX()))
				* (180.0 / Math.PI);
		if (minAng < 0) {
			minAng += 360.0;
		}
		return minAng;
	}
	
	protected boolean facingDest(double angle) {
		return Math.abs(angle - odometer.getAng()) < DEG_ERR;
	}
	
	protected boolean checkIfDone(double x, double y) {
		return Math.abs(x - odometer.getX()) < CM_ERR
				&& Math.abs(y - odometer.getY()) < CM_ERR;
	}

	
	private boolean checkObstacle()
	{
		if(getFilteredData() < 15)
			{
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
	
	private boolean checkEmergency() {
			
		return (checkObstacle()) ;
	
	}
	
	private void updateTravel() {
		double minAng;

		minAng = getDestAngle(destx, desty);
		/*
		 * Use the BasicNavigator turnTo here because 
		 * minAng is going to be very small so just complete
		 * the turn.
		 */
		turnTo(minAng);
		this.setSpeeds(FAST, FAST);
		
	//	TravelTo(destx,desty);
	}
	public boolean isTravelling() {
		return isNavigating;
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
	public void run() {
		obstacleavoid avoidance = null;
	//	searching search = null;
		state = State.INIT;
		while (jumpout==false) {
		
			switch (state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				}
				break;
			case TURNING:
				/*
				 * Note: you could probably use the original turnTo()
				 * from BasicNavigator here without doing any damage.
				 * It's cheating the idea of "regular and periodic" a bit
				 * but if you're sure you never need to interrupt a turn there's
				 * no harm.
				 * 
				 * However, this implementation would be necessary if you would like
				 * to stop a turn in the middle (e.g. if you were travelling but also
				 * scanning with a sensor for something...)
				 * 
				 */
				double destAngle = getDestAngle(destx, desty);
				turnTo(destAngle);
				if(facingDest(destAngle)){
					setSpeeds(0,0);
					state = State.TRAVELLING;
				}
				break;
			case TRAVELLING:
	            if(collide())
	            {
	        
	            	leftMotor.rotate(convertDistance(2.2,-10),true);
	    			rightMotor.rotate(convertDistance(2.2,-10),false);
	    			leftMotor.stop(true);
	            	rightMotor.stop(false);
	            	state=State.EMERGENCY;
	            	avoidance = new obstacleavoid(leftMotor,rightMotor,sensorMotor,odometer,usSensor, usData,touch1,touch2,touchSample1,touchSample2);
					avoidance.start();
	            	
	            }
				if (checkEmergency()) { // order matters!
					state = State.EMERGENCY;
					leftMotor.stop(true);
					rightMotor.stop(false);
					avoidance = new obstacleavoid(leftMotor,rightMotor,sensorMotor,odometer,usSensor, usData,touch1,touch2,touchSample1,touchSample2);
					avoidance.start();
				} else 
					if (!checkIfDone(destx, desty)) {
					updateTravel();
				} else { // Arrived!
					setSpeeds(0, 0);
					isNavigating = false;
					state=State.INIT;
				//	state = State.SEARCHING;
				//	search = new searching(leftMotor, rightMotor,sensorMotor,odometer, usSensor,usData,touch1, touch2,
				//		touchSample1,touchSample2);
				//	search.start();
							
				}
				break;
				
	/*		case SEARCHING:
				if (searching.isfound()) {
					state = State.INIT;
				}
				break;
	*/			
			case EMERGENCY:
				if (avoidance.resolved()) {
					state = State.TURNING;
				}
				break;
			}
	/*		
			search = new searching(leftMotor, rightMotor,sensorMotor,odometer, usSensor,usData,touch1, touch2,
					touchSample1,touchSample2);
				search.start();
				*/
		}
	}
}
		

	