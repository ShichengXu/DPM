package finalproject;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigation {
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
	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
	private static double theta = 0;
	private static double destx, desty;
	private static double currentX, currentY, currentT, goToX, goToY;
	final static double DEG_ERR = 1.0, CM_ERR = 1.0;
	private static double LEFTR = 2.2, RIGHTR = 2.2, WIDTH = 15.3;
	private static boolean isNavigating = true;
	enum State {INIT, TURNING, TRAVELLING, EMERGENCY};
	State state;
	private boolean jumpOut;

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
		double error = (angle - this.odometer.getAng()+360)%360;
				if(error<180){
			leftMotor.rotate(-convertAngle(2.2, 15.3, error), true);
			rightMotor.rotate(convertAngle(2.2, 15.3, error), false);
		}else{
			leftMotor.rotate(convertAngle(2.2, 15.3, 360-error), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, 360-error), false);
		}
		

//			if (error < -180.0) {
//				leftMotor.rotate(-convertAngle(2.2, 15.5, angle), true);
//				rightMotor.rotate(convertAngle(2.2, 15.5, angle), false);
//			} else if (error < 0.0) {
//				leftMotor.rotate(convertAngle(2.2, 15.5, angle), true);
//				rightMotor.rotate(-convertAngle(2.2, 15.5, angle), false);
//			} else if (error > 180.0) {
//				leftMotor.rotate(convertAngle(2.2, 15.5, angle), true);
//				rightMotor.rotate(-convertAngle(2.2, 15.5, angle), false);
//			} else {
//				leftMotor.rotate(-convertAngle(2.2, 15.5, angle), true);
//				rightMotor.rotate(convertAngle(2.2, 15.5, angle), false);
//			}
		

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
		
		 if(getFilteredData() < 15 && Math.abs(FinalProject.targetXYT[0]-odometer.getX())>30 &&
					Math.abs(FinalProject.targetXYT[1]-odometer.getY())>30)
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
		double minerror = minAng - odometer.getAng();
		if(minerror>3){ 
		turnTo(minAng);
		}else{
		this.setSpeeds(FAST, FAST);
		}
	//	TravelTo(destx,desty);
	}
	public boolean isTravelling() {
		return isNavigating;
	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	
	
	public void doNavigation(boolean avoid) {
		obstacleavoid avoidance = null;
		sensorMotor.rotateTo(0,false);
	//	searching search = null;
		state = State.INIT;
		jumpOut=false;
		while (jumpOut==false) {
		
			switch (state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				}
				else
				{
					jumpOut=true;
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
				if(avoid)
				{
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
				
							
				}
				break;
				}
				else if(avoid==false)
				{
					if (!checkIfDone(destx, desty)) {
						updateTravel();
					} else { // Arrived!
						setSpeeds(0, 0);
						isNavigating = false;
						state=State.INIT;
					}
					break;
				}
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
		

	