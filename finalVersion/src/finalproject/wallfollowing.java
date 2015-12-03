package finalproject;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * use wallfollowing mechanism to travel to the corner which is 
 * the closest to opponent zone.
 * @author Ryan Xu
 */
public class wallfollowing {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider touch1, touch2;
	private float[] touchSample1, touchSample2;
	private Odometer odometer;
	public static int ROTATION_SPEED = 200, maxSpeed = 400;
	public double theta, tempDegree, tempX, tempY;
	private static boolean solved = false;
	private int bandCenter = 25, bandwidth = 3;
	private int motorLow = 200, motorHigh = 400, motorStraight = 300;
	private int FILTER_OUT = 20;
	private int filterControl = 0;
	private double distance;
	private int error;
	private double[] hitterArray;
	private double xSave, ySave, angleSave;
	public wallfollowing(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			SampleProvider usSensor, float[] usData, SampleProvider touch1,
			SampleProvider touch2, float[] touchSample1, float[] touchSample2,
			EV3LargeRegulatedMotor sensorMotor) {
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		this.touchSample1 = touchSample1;
		this.touchSample2 = touchSample2;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.touch1 = touch1;
		this.touch2 = touch2;

	}

	/**
	 * follow walls numbered 1,2,3 and 4 and travel to one of four corners
	 * @param path 
	 *  --------3----------
	 *  |X4             X3|
	 *  4                 3
	 *  |X1             X2|
	 *  --------1----------
	 * @param targetX target coordinate
	 * @param targetY
	 */
	public void doWallfollowing(int path, double targetX, double targetY) {

		synchronized (this) {
			leftMotor.rotate(-convertAngle(2.2, 15, 89.8), true);
			rightMotor.rotate(convertAngle(2.2, 15, 89.8), false);
		}
		while (arrive(path, targetX, targetY) == false) {
			if (collide() == true) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				solved = false;
				synchronized (this) {
					Sound.beep();
					leftMotor.rotate(convertDistance(2.2, -10), true);
					rightMotor.rotate(convertDistance(2.2, -10), false);
				}
				synchronized (this) {
					leftMotor.rotate(-convertAngle(2.2, 15, 89.8), true);
					rightMotor.rotate(convertAngle(2.2, 15, 89.8), false);
				}
				xSave = odometer.getX();
				ySave = odometer.getY();
				angleSave = odometer.getAng();

				while (solved == false) {
					if (getFilteredData() > 100 && filterControl < FILTER_OUT) {

						filterControl++;
					} else if (getFilteredData() > 100) {
						// set distance to this.distance
						distance = 100;
					} else {
						// distance went below 100, therefore reset everything.
						filterControl = 0;
						distance = getFilteredData();
					}

					error = (int) (distance - bandCenter);
					// set the error as the difference between distance and
					// bandCenter
					if (error > -bandwidth && error < bandwidth)
					// happens when error is within the tolerance (bandwidth in
					// this case)
					{

						setSpeeds(motorStraight, motorStraight);
						// both wheels rotate at the same speed.
					}
					if (error < 0) {
						setSpeeds(motorLow, motorStraight);
					}

					else if (error > 20) {
						setSpeeds(400, 100);
					} else if (error > 10 && error <= 20) {
						setSpeeds(400, 200);
					}

					else {
						setSpeeds(motorHigh, motorStraight);
					}

					if (checkSolved(path, angleSave, odometer.getAng(), xSave,
							odometer.getX(), ySave, odometer.getY())) {
						leftMotor.stop(true);
						rightMotor.stop(false);
						solved = true;
						Sound.beepSequence();
					}
				}
				synchronized (this) {
					leftMotor.rotate(-convertAngle(2.2, 15, 89.8), true);
					rightMotor.rotate(convertAngle(2.2, 15, 89.8), false);
					Sound.beepSequenceUp();
				}
			} else if (collide() == false) {
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

				error = (int) (distance - bandCenter);
				// set the error as the difference between distance and
				// bandCenter
				if (error > -bandwidth && error < bandwidth)
				// happens when error is within the tolerance (bandwidth in this
				// case)
				{

					setSpeeds(motorStraight, motorStraight);
					// both wheels rotate at the same speed.
				}
				if (error < 0) {
					setSpeeds(motorLow, motorStraight);
				} else if (error > 0) {
					setSpeeds(motorHigh, motorStraight);
				}
			}
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		Sound.beepSequence();
		Sound.beepSequenceUp();

		hitTheWall();
		Sound.buzz();
		synchronized (this) {
			leftMotor.rotate(convertAngle(2.2, 15.3, 89.8), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, 89.8), false);

		}
		hitTheWall();
		Sound.buzz();
		Sound.buzz();
		synchronized (this) {
			leftMotor.rotate(convertAngle(2.2, 15.3, 180), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, 180), false);
		}
		System.exit(0);

	}

	/**
	 * if obstacle avoidance is completed
	 * @param select which path is the robot taking
	 * @param anglePre angle before avoiding
	 * @param anglePost angle after avoiding
	 * @param xPre x coordinate before avoiding
	 * @param xPost x coordinate after avoiding
	 * @param yPre y coordinate before avoiding
	 * @param yPost y coordinate after avoiding
	 * @return boolean true or false
	 */
	public boolean checkSolved(int select, double anglePre, double anglePost,
			double xPre, double xPost, double yPre, double yPost) {
		if (select == 1) {
			if (Math.abs((anglePost - anglePre) - 180) < 30
					&& Math.abs(yPost - yPre) < 10 || collide() == true) {
				return true;
			}
		}
		if (select == 2) {
			if (Math.abs((anglePost - anglePre) + 180) < 30
					&& Math.abs(xPost - xPre) < 10 || collide() == true) {
				return true;
			}
		}
		return false;
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

	/**
	 * hit the wall to update odomter.
	 */
	public void hitTheWall() {
		setSpeeds(350, 350);
		while (true) {
			touch1.fetchSample(touchSample1, 0);
			touch2.fetchSample(touchSample2, 0);
			if (touchSample1[0] == 1 && touchSample2[0] == 1) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				tempDegree = hitterDegHelper(odometer.getAng());
				tempX = odometer.getX();
				tempY = odometer.getY();
				hitterArray = hitterDisHelper(tempX, tempY, tempDegree);
				odometer.setPosition(new double[] { hitterArray[0],
						hitterArray[1], tempDegree }, new boolean[] { true,
						true, true });
				synchronized (this) {
					leftMotor.rotate(convertDistance(2.2, -20), true);
					rightMotor.rotate(convertDistance(2.2, -20), false);
				}

				break;
			}

		}
	}
/**
 * provide coordinates and angle for hitTheWall method
 * @param xcoord x coordinate
 * @param ycoord y coordinate
 * @param angle angle of impact
 * @return x y and angle after impact with the wall
 */
	public double[] hitterDisHelper(double xcoord, double ycoord, double angle) {
		double[] result = new double[2];

		if (angle == 90) {
			ycoord = 319.5;
			result[0] = xcoord;
			result[1] = ycoord;
		} else if (angle == 180) {
			xcoord = -19.5;
			result[0] = xcoord;
			result[1] = ycoord;
		} else if (angle == 270) {
			ycoord = -19.5;
			result[0] = xcoord;
			result[1] = ycoord;
		} else if (angle == 0) {
			xcoord = 319.5;
			result[0] = xcoord;
			result[1] = ycoord;
		}
		return result;
	}
/**
 * check if wallfollowing is done
 * @param path path from 1-4
 * @param targetX target x coordinate
 * @param targetY target y coordinate
 * @return true->done false->no
 */
	public boolean arrive(int path, double targetX, double targetY) {
		if (path == 1 || path == 3) {
			if (Math.abs(odometer.getX() - targetX) < 30) {
				return true;
			}
		} else if (path == 2 || path == 4) {
			if (Math.abs(odometer.getY() - targetY) < 30) {
				return true;
			}

		}
		return false;
	}

	/**
	 * update angle after collision base on odometer
	 * @param angle angle when collide
	 * @return angle of 90,180,270 or 0
	 */
	public double hitterDegHelper(double angle) {
		if (angle > 45 && angle <= 135) {
			return 90;
		} else if (angle > 135 && angle <= 225) {
			return 180;
		} else if (angle > 225 && angle <= 315) {
			return 270;
		} else {
			return 0;
		}
	}
/**
 * check if left and right touch sensor collides with obstacles.
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

	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		if (distance > 100) {
			distance = 100;
		}
		return distance;
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}
