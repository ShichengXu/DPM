package finalproject;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/**
 * obstacle avoidance using wall following mechanism running as a thread
 * @author Ryan Xu
 */
public class obstacleavoid extends Thread {
	private static EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;
	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;
	private static final int ROTATE_SPEED = 100;
	Navigation nav;
	boolean safe;
	private int bandCenter = 22;
	private int motorStraight = 200, FILTER_OUT = 20;
	private double distance;
	private int filterControl;
	private double proportion = 20;
	private double perror;
	private double amperror;
	private double angleSave;
	private double distLeft, distRight;
	private int select = 1;
	private static final int maxSpeed = 400;

	public obstacleavoid(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor sensorMotor, Odometer odometer,
			SampleProvider usSensor, float[] usData) {

		this.odometer = odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		safe = false;

	}
/**
 * this body of the obstacle avoidance. We used wall follower codes to get around obstacles
 * and touch sensors act as insurance.
 */
	public void run() {
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);

		sensorMotor.rotate(45);
		Delay.msDelay(500);
		distLeft = getFilteredData();
		sensorMotor.rotate(-90);
		Delay.msDelay(500);
		distRight = getFilteredData();
		sensorMotor.rotate(45);

		if (distLeft >= distRight) {
			select = 0;
		} else if (distRight >= distLeft) {
			select = 2; // turn right
		}

		if (select == 0) {
			synchronized (this) {
				leftMotor.setSpeed(200);
				rightMotor.setSpeed(200);
			}

			sensorMotor.setAcceleration(1000);
			angleSave = odometer.getAng();
			leftMotor.rotate(-convertAngle(2.2, 15, 89.8), true);
			rightMotor.rotate(convertAngle(2.2, 15, 89.8), false);
			sensorMotor.rotate(-85);

			while (safe == false) {
				if (getFilteredData() > 100 && filterControl < FILTER_OUT) {
					// bad value, do not set the distance var, however do
					// increment the filter value
					filterControl++;
				} else if (getFilteredData() > 100) {
					// set distance to this.distance
					distance = 100;
				} else {
					// distance went below 100, therefore reset everything.
					filterControl = 0;
					distance = getFilteredData();
				}

				perror = distance - bandCenter;
				amperror = perror * proportion;
				if (distance > 60) {
					// if distance >80, the robot is about to do a U-turn

					setSpeed(maxSpeed, motorStraight);

					// right wheel will speed up in order to turn left

				}
			
				else {
					// general proportion error correction

					if ((motorStraight + amperror) > maxSpeed) {

						setSpeed(maxSpeed, motorStraight);

					}
					// left wheel remains constant speed
					else {
						setSpeed((int) (motorStraight + amperror),
								motorStraight);
					}
					// right wheel will continuously correcting its speed
					// according to errors.

				}

				if (CloseTo(rightDegreeHelper(angleSave, odometer.getAng()),
						-30)) {
					sensorMotor.rotate(85);
					safe = true;
				}

			}
		}

		if (select == 2) {
			setSpeed(ROTATE_SPEED, ROTATE_SPEED);
			sensorMotor.setAcceleration(1000);
			angleSave = odometer.getAng();
			leftMotor.rotate(convertAngle(2.2, 15, 89.8), true);
			rightMotor.rotate(-convertAngle(2.2, 15, 89.8), false);
			sensorMotor.rotate(85);

			while (safe == false) {
				if (getFilteredData() > 100 && filterControl < FILTER_OUT) {
					// bad value, do not set the distance var, however do
					// increment the filter value
					filterControl++;
				} else if (getFilteredData() > 100) {
					// set distance to this.distance
					distance = 100;
				} else {
					// distance went below 100, therefore reset everything.
					filterControl = 0;
					distance = getFilteredData();
				}

				perror = distance - bandCenter;
				amperror = perror * proportion;
				if (distance > 60) {
					// if distance >80, the robot is about to do a U-turn
					setSpeed(motorStraight, maxSpeed);

					// right wheel will speed up in order to turn left

				}

				else {
					// general proportion error correction

					if ((motorStraight + amperror) > maxSpeed) {

						setSpeed(motorStraight, maxSpeed);

					}
					// left wheel remains constant speed
					else {

						setSpeed(motorStraight,
								(int) (motorStraight + amperror));

					}
					// right wheel will continuously correcting its speed
					// according to errors.

				}

				if (CloseTo(leftDegreeHelper(angleSave, odometer.getAng()), 30)) {
					sensorMotor.rotate(-85);
					safe = true;
				}

			}

		}
	}

	public synchronized void setSpeed(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		leftMotor.forward();
		rightMotor.forward();
	}
/**
 * a helper method to round up angles when we are doing calculations on angles
 * WHEN THE ROBOT IS AVOIDING OBSTACLES BY TURNING TO ITS RIGHT
 * @param angle1 target angle
 * @param angle2 original angle
 * @return angle that can be used for further determination methods.
 */
	public double leftDegreeHelper(double angle1, double angle2) {
		if (angle2 - angle1 <= 0) {
			return (angle2 - angle1 + 360);
		}
		return angle2 - angle1;
	}
/**
 * a helper method to round up angles when we are doing calculations on angles
 * WHEN THE ROBOT IS AVOIDING OBSTACLES BY TURNING TO ITS RIGHT
 * @param angle1 target angle
 * @param angle2 original angle
 * @return angle that can be used for further determination methods.
 */
	public double rightDegreeHelper(double angle1, double angle2) {
		if (angle2 - angle1 >= 180) {
			return (angle2 - angle1 - 360);
		}
		return angle2 - angle1;
	}
/**
 * check if this emergency case is solved
 * @return boolean true or false
 */
	public boolean resolved() {
		return safe;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
/**
 * a helper method to check if two input numbers are at most 16 away from each other 
 * @param a first input
 * @param b seond input
 * @return boolean true or false
 */
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
		if (distance > 80) {
			distance = 80;
		}

		return distance;
	}

}