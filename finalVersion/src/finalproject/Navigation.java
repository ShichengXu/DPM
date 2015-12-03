package finalproject;


import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.robotics.SampleProvider;

public class Navigation {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor sensorMotor;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider touch1, touch2;
	private float[] touchSample1, touchSample2;
	private Odometer odometer;

	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;

	private static double destx, desty;

	final static double DEG_ERR = 1.0, CM_ERR = 1.0;

	private static boolean isNavigating = true;

	enum State {
		INIT, TURNING, TRAVELLING, EMERGENCY
	};

	State state;
	private boolean jumpOut;

	public Navigation(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			SampleProvider usSensor, float[] usData, SampleProvider touch1,
			SampleProvider touch2, float[] touchSample1, float[] touchSample2,
			EV3LargeRegulatedMotor sensorMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		this.touchSample1 = touchSample1;
		this.touchSample2 = touchSample2;
		this.touch1 = touch1;
		this.touch2 = touch2;
		this.sensorMotor = sensorMotor;
	}

	/**
	 * pass x and y coordinates as target information
	 * @param x x coordinate
	 * @param y y coordinate
	 */
	public void TravelTo(double x, double y) {
		destx = x;
		desty = y;
		isNavigating = true;
	}

	/**
	 * will turn to a certain angle.
	 * @param angle desired angle wanted to turnto
	 */
	public void turnTo(double angle) {
		double error = (angle - this.odometer.getAng() + 360) % 360;
		if (error < 180) {
			leftMotor.rotate(-convertAngle(2.2, 15.3, error), true);
			rightMotor.rotate(convertAngle(2.2, 15.3, error), false);
		} else {
			leftMotor.rotate(convertAngle(2.2, 15.3, 360 - error), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, 360 - error), false);
		}


	}

	/**
	 * set speeds and move forward or backwards relatively.
	 * @param lSpd leftmotor speed
	 * @param rSpd rightmotor speed
	 */
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

	/**
	 * calculate angle it needs to turn to head to destination
	 * @param x target x coordinate
	 * @param y target y coordinate
	 * @return desired angle
	 */
	protected double getDestAngle(double x, double y) {
		double minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX()))
				* (180.0 / Math.PI);
		if (minAng < 0) {
			minAng += 360.0;
		}
		return minAng;
	}

	/**
	 * check if the odometer angle matches the input one
	 * @param angle desired angle
	 * @return boolean true or false
	 */
	protected boolean facingDest(double angle) {
		return Math.abs(angle - odometer.getAng()) < DEG_ERR;
	}
/**
 * check if the traveling progress is finished using odomter
 * @param x desired x coordinate
 * @param y desired y coordinate
 * @return boolean true or false
 */
	protected boolean checkIfDone(double x, double y) {
		return Math.abs(x - odometer.getX()) < CM_ERR
				&& Math.abs(y - odometer.getY()) < CM_ERR;
	}

	/**
	 * check if there is obstacle ahead (15 cm)
	 * @return boolean true or false
	 */
	private boolean checkObstacle() {

		if (getFilteredData() < 15
				&& Math.abs(FinalProject.targetXYT[0] - odometer.getX()) > 30
				&& Math.abs(FinalProject.targetXYT[1] - odometer.getY()) > 30) {
			return true;
		}
		return false;
	}
/**
 * clip us data which is bigger than 100
 * @return distance
 */
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		if (distance > 100) {
			distance = 100;
		}

		return distance;
	}

	/**
	 * check if touch sensor was collided with obstacles, touch1-left touch sensor
	 * touch2- right touch sensor
	 * @return boolean true or false
	 */
	private boolean collide() {
		touch1.fetchSample(touchSample1, 0);
		touch2.fetchSample(touchSample2, 0);
		if (touchSample1[0] == 1 || touchSample2[0] == 1) {
			return true;
		}
		return false;
	}
/**
 * check if the robot encounters any obstacles
 * @return boolean true or false
 */
	private boolean checkEmergency() {

		return (checkObstacle());

	}

	/**
	 * when angle is off which is beyond the error range, this method
	 * will update the angle to where it's supposed to be
	 */
	private void updateTravel() {
		double minAng;

		minAng = getDestAngle(destx, desty);
		/*
		 * Use the BasicNavigator turnTo here because minAng is going to be very
		 * small so just complete the turn.
		 */
		double minerror = minAng - odometer.getAng();
		if (minerror > 3) {
			turnTo(minAng);
		} else {
			this.setSpeeds(FAST, FAST);
		}
		// TravelTo(destx,desty);
	}

	/**
	 * keep tracks of the robot status wheter it is traveling or not
	 * @return true of false
	 */
	public boolean isTravelling() {
		return isNavigating;
	}

	/**
	 * convert desired distance in number of rotation in wheels
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
/**
 * finite state with 4 states. 
 * @param avoid if avoid is false, it will travel without obstacle avoidance. 
 * if avoid is true, it will travel with obstacle avoidance.
 */
	public void doNavigation(boolean avoid) {
		obstacleavoid avoidance = null;
		sensorMotor.rotateTo(0, false);
		// searching search = null;
		state = State.INIT;
		jumpOut = false;
		while (jumpOut == false) {

			switch (state) {
			case INIT:
				if (isNavigating) {
					state = State.TURNING;
				} else {
					jumpOut = true;
				}
				break;
			case TURNING:
				/*
				 * Note: you could probably use the original turnTo() from
				 * BasicNavigator here without doing any damage. It's cheating
				 * the idea of "regular and periodic" a bit but if you're sure
				 * you never need to interrupt a turn there's no harm.
				 * 
				 * However, this implementation would be necessary if you would
				 * like to stop a turn in the middle (e.g. if you were
				 * travelling but also scanning with a sensor for something...)
				 */
				double destAngle = getDestAngle(destx, desty);
				turnTo(destAngle);
				if (facingDest(destAngle)) {
					setSpeeds(0, 0);
					state = State.TRAVELLING;
				}
				break;
			case TRAVELLING:
				if (avoid) {
					if (collide()) {

						leftMotor.rotate(convertDistance(2.2, -10), true);
						rightMotor.rotate(convertDistance(2.2, -10), false);
						leftMotor.stop(true);
						rightMotor.stop(false);
						state = State.EMERGENCY;
						avoidance = new obstacleavoid(leftMotor, rightMotor,
								sensorMotor, odometer, usSensor, usData);

						avoidance.start();

					}
					if (checkEmergency()) { // order matters!
						state = State.EMERGENCY;
						leftMotor.stop(true);
						rightMotor.stop(false);
						avoidance = new obstacleavoid(leftMotor, rightMotor,
								sensorMotor, odometer, usSensor, usData);
						avoidance.start();
					} else if (!checkIfDone(destx, desty)) {
						updateTravel();
					} else { // Arrived!
						setSpeeds(0, 0);
						isNavigating = false;
						state = State.INIT;

					}
					break;
				} else if (avoid == false) {
					if (!checkIfDone(destx, desty)) {
						updateTravel();
					} else { // Arrived!
						setSpeeds(0, 0);
						isNavigating = false;
						state = State.INIT;
					}
					break;
				}
				/*
				 * case SEARCHING: if (searching.isfound()) { state =
				 * State.INIT; } break;
				 */
			case EMERGENCY:
				if (avoidance.resolved()) {
					state = State.TURNING;
				}
				break;
			}
		}
	}
}
