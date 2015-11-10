package finalproject;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static Odometer odo;
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 100;
	private static double theta = 0;
	private static double currentX, currentY, currentT, goToX, goToY;
	private static double LEFTR = 2.1, RIGHTR = 2.1, WIDTH = 15;
	private static boolean isNavigating = true;
//	private static boolean isTraveling = true, isLeftTraveling = true,
//			isRightTraveling = true;

	
	public Navigation(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, Odometer odo) {
		this.theta = theta;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odo = odo;
	}

	private static boolean CloseTo(double a, double b) {
		if (b - 1 < a && a < b + 1) { // if a is within 2cm away from b return
										// true otherwise return false.
			return true;
		}
		return false;
	}

	public static void TravelTo(double x, double y) {
		while (isNavigating) {

			currentX = odo.getX();
			currentY = odo.getY();
			currentT = odo.getAng();
			goToX = x - currentX;
			goToY = y - currentY;
			if (goToY > 0) {
				theta = Math.atan(goToX / goToY) * 180 / Math.PI;
			} else if (goToX > 0 && goToY < 0) {
				theta = (Math.atan(goToX / goToY) + Math.PI) * 180 / Math.PI;
			} else if (goToX < 0 && goToY < 0) {
				theta = (Math.atan(goToX / goToY) - Math.PI) * 180 / Math.PI;
			}

			LCD.drawString("theta " + theta, 0, 6);

			turnTo(theta);
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();
			if ((CloseTo(x, odo.getX())) && (CloseTo(y, odo.getY()))) {
				leftMotor.stop();
				rightMotor.stop();
				isNavigating = false;
			}

		}
	}

	// public static boolean isNavigating(){
	// return isNavigating;
	// }

	public static void turnTo(double theta) {
		if (theta > 3 && theta - odo.getAng() > 15) { //
			while (Math.abs(odo.getAng() - theta) > 3) {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.forward();
				rightMotor.backward();
			}
		}
		if (CloseTo(theta, 0)) // robot turn clockwise till odometer theta
								// matches calculated theta.
		{
			while (Math.abs(odo.getAng() - theta) > 3) {
				{
					leftMotor.setSpeed(ROTATE_SPEED);
					rightMotor.setSpeed(ROTATE_SPEED);
					leftMotor.forward();
					rightMotor.backward();
				}
			}
		}
		if (theta < -3 && Math.abs(theta - odo.getAng()) > 15) { // make
																		// the
																		// robot
																		// turn
																		// counterclockwise
			while (Math.abs(odo.getAng() - theta) > 3) {
				leftMotor.setSpeed(ROTATE_SPEED);
				rightMotor.setSpeed(ROTATE_SPEED);
				leftMotor.backward();
				rightMotor.forward();
			}
		}

		if (Math.abs(theta - odo.getAng()) > 2
				&& Math.abs(theta - odo.getAng()) < 15) // minor changes
																// in direction
		{
			leftMotor.setSpeed((int) (FORWARD_SPEED + (theta - odo
					.getAng())) * 5);
			rightMotor.setSpeed((int) (FORWARD_SPEED - (theta - odo
					.getAng())) * 5);
		}

	}

	public void run() {
		TravelTo(60, 30); // travel to 4 way points, reinitialize isNavigating
							// to true after each path is completed.
		isNavigating = true;
		TravelTo(30, 30);
		isNavigating = true;
		TravelTo(30, 60);
		isNavigating = true;
		TravelTo(60, 0);

	}

}