package finalproject;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.*;

public class obstacleavoid extends Thread {
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;

	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 100;
	private static double theta = 0, deltaTheta = 0;
	private static double currentX, currentY, currentT, goToX, goToY,
			saveTheta;
	private static double LEFTR = 2.1, RIGHTR = 2.1, WIDTH = 15;
	private static boolean isNavigating = true, good = true;
	private static boolean isTraveling = true, isLeftTraveling = true,
			isRightTraveling = true;
	private static double counter, error, BANDCENTER = 32;

	public obstacleavoid(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.theta = theta;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.usData = usData;
	}

	private static boolean CloseTo(double a, double b) {
		if (b - 1 < a && a < b + 1) { // if a is within 1cm away from b return
										// true otherwise return false.
			return true;
		}
		return false;
	}

	public static void TravelTo(double x, double y) {
		while (isNavigating) {

			currentX = odometer.getX(); // calculate delta x and delta y the
										// robot needs to travel
			currentY = odometer.getY(); // and those are used to calculate the
										// angle our robot needs to turn
			currentT = odometer.getAng();
			goToX = x - currentX;
			goToY = y - currentY;
			if (goToY > 0) // three scenarios to determine the turning angle
							// theta(from tutorial slides).
			{
				theta = Math.atan(goToX / goToY) * 180 / Math.PI;
			} else if (goToX > 0 && goToY < 0) {
				theta = (Math.atan(goToX / goToY) + Math.PI) * 180 / Math.PI;
			} else if (goToX < 0 && goToY < 0) {
				theta = (Math.atan(goToX / goToY) - Math.PI) * 180 / Math.PI;
			}

			LCD.drawString("theta " + theta, 0, 6);

			turnTo(theta); // turn to the desired theta.
		}
	}



	public static void turnTo(double theta) { // turn to method, general idea is
												// make the robot turn until the
												// desired theta is met.

		if (theta > 3 && theta - odometer.getAng() > 10) {
			while (Math.abs(odometer.getAng() - theta) > 3) {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.forward();
				rightMotor.backward();
			}
		}

		if (theta > 3 && theta - odometer.getAng() < -10) {
			while (Math.abs(odometer.getAng() - theta) > 3) {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.backward();
				rightMotor.forward();
			}
		}

		if (CloseTo(theta, 0)) {
			while (Math.abs(odometer.getAng() - theta) > 3) {
				{
					leftMotor.setSpeed(ROTATE_SPEED);
					rightMotor.setSpeed(ROTATE_SPEED);
					leftMotor.forward();
					rightMotor.backward();
				}
			}
		}
		if (theta < -3 && theta - odometer.getAng() < -10) {
			while (Math.abs(odometer.getAng() - theta) > 3) {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.backward();
				rightMotor.forward();
			}
		}

		if (Math.abs(theta - odometer.getAng()) > 2
				&& Math.abs(theta - odometer.getAng()) < 15) // minor changes
																// along the way
																// if the degree
																// is not off by
																// too much
		{
			leftMotor.setSpeed((int) (FORWARD_SPEED + (theta - odometer
					.getAng())) * 5);
			rightMotor.setSpeed((int) (FORWARD_SPEED - (theta - odometer
					.getAng())) * 5);
		}

	}

	public void run() { // travel to 2 way points.
		TravelTo(0, 60);
		isNavigating = true;
		TravelTo(60, 0);

	}

}