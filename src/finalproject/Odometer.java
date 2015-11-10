package finalproject;

import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * this odometer class runs as a separated thread. Keep tracks of the position of robot
 * @author DPM, customized by Ryan Xu
 */

public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;
	private Object lock;

	/**
	 * constructs the odometer with motors and time intervals for listener.
	 * @param leftMotor 
	 * @param rightMotor
	 * @param INTERVAL time interval odometer pull the data
	 * @param autostart
	 */
	
	public Odometer(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, int INTERVAL, boolean autostart) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// default values, modify for your robot
		this.rightRadius = 2.1;
		this.leftRadius = 2.1;
		this.width = 15.1;

		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL
					: DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}

	// functions to start/stop the timerlistener
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}

	public void start() {
		if (this.timer != null)
			this.timer.start();
	}

	/*
	 * Calculates displacement and heading as title suggests
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI
				/ 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}

	/*
	 * Recompute the odometer values using the displacement and heading changes
	 */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta * 180 / Math.PI; // shows theta in degree
		}
	}
	/**
	 * get x coordinate
	 * @return x coordinate
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * get y coordinate
	 * @return y coordinate
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * get angle
	 * @return angle
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	
	/**
	 * set position for the odometer
	 * @param position as a string of coordinates
	 * @param update as a string of booleans: true as update and false as not update
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * get current x,y,angle 
	 * @param position x=[0],y=[1],angle=[2]
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}

	// accessors to motors
	public EV3LargeRegulatedMotor[] getMotors() {
		return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
	}

	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * helper methods
	 * @param angle if it is less than 0, round up to 
	 * @return a corrected angle from 0 to 360 degrees
	 */
	
	
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	/**
	 * helper method that corrects angles those are more than 360 degrees and find minimum angle
	 * @param a is the first input
	 * @param b is the second input
	 * @return a corrected angle
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
}
