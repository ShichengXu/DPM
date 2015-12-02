package finalproject;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class searching {
	private static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	private static final EV3MediumRegulatedMotor claw = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private static EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;
	Navigation nav;
	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider touch1, touch2;
	private float[] touchSample1, touchSample2;
	private double distance;
	private int filterControl;
	private double tempDist;
	private static int colorfilterControl = 0;
	private double proportion = 5;
	private double safeZone = 10;
	private double perror;
	private double amperror;
	private int cruisingSpeed = 200, FILTER_OUT = 80, motorStraight = 200;
	private static final int maxSpeed = 350;
	private int bandCenter = 30;
	static boolean found, isObject;
	private double minDis = 100, targetAng;
	private static int colorTemp = 0, colourFilter = 100;
	private int targetID = 1;
	private double opzBL_X, opzBL_Y, opzTR_X, opzTR_Y;
	private int i, j, k;
	private int counterXd, counterYd;
	private double xSave, ySave, xhorizD;

	// ArrayList <Double> al = new ArrayList<Double>();

	public searching(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor sensorMotor, Odometer odometer, SampleProvider usSensor, float[] usData,
			SampleProvider touch1, SampleProvider touch2, float[] touchSample1, float[] touchSample2, Navigation nav) {
		this.nav = nav;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		this.odometer = odometer;
		this.usSensor = usSensor;
		this.usData = usData;
		this.touchSample1 = touchSample1;
		this.touchSample2 = touchSample2;
		this.touch1 = touch1;
		this.touch2 = touch2;
		found = false;

	

	}

	public void doSearching() {
		opzBL_X = FinalProject.opponentHomeZoneBL[0];    //opponent zone BL and TR coordinate
		opzBL_Y = FinalProject.opponentHomeZoneBL[1];   
		opzTR_X = FinalProject.opponentHomeZoneTR[0];
		opzTR_Y = FinalProject.opponentHomeZoneTR[1];
        sensorMotor.rotate(400,false);
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		for (i = 0; i < (opzTR_Y - opzBL_Y) / 30; i++) {

			if (found) {
				Sound.beepSequenceUp();
				return;
			}
			xSave = odometer.getX();
			ySave = odometer.getY();
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			sensorMotor.rotateTo(-90, false);
			isObject = false;
			synchronized (this) {
				leftMotor.forward();
				rightMotor.forward();
			}
			while (closeTo(odometer.getY(), ySave + 30) == false) {

				if (getFilteredData() < 20) {
					synchronized (this) {
						leftMotor.rotate(convertDistance(2.2, 13), true);
						rightMotor.rotate(convertDistance(2.2, 13), false);
					}
					xhorizD = getFilteredData();
					isObject = true;
					break;
				}
			}

			synchronized (this) {
				leftMotor.stop(true);
				rightMotor.stop(false);
			}

			if (isObject == true) {
				checkandgrab(85, xhorizD - 8);
				if (found == true) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					System.exit(0);
					//break;
				} else {

				
			//		odometer.setPosition(new double[] { 0.0, 0.0, 85.0 }, new boolean[] { false, false, true });

					nav.TravelTo(xSave, ySave + 30);
					Sound.beepSequence();

					nav.doNavigation(false);
					nav.turnTo(90);
				}
				

			}
			else if(isObject == false){
				sensorMotor.rotateTo(0, false);
				synchronized (this) {
					leftMotor.stop(true);
					rightMotor.stop(false);
				}

			}

		}

		synchronized (this) {

			leftMotor.rotate(convertAngle(2.2, 15.3, 89.8), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, 89.8), false);
		}

		for (j = 0; j < (opzTR_X - opzBL_X) / 30; j++) {
			if (found) {
				Sound.beepSequenceUp();
				return;
			}
			xSave = odometer.getX();
			ySave = odometer.getY();
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			sensorMotor.rotateTo(-90, false);
			isObject = false;
			synchronized (this) {
				leftMotor.forward();
				rightMotor.forward();
			}
			while (closeTo(odometer.getX(), xSave + 30) == false) {

				if (getFilteredData() < 20) {
					synchronized (this) {
						leftMotor.rotate(convertDistance(2.2, 13), true);
						rightMotor.rotate(convertDistance(2.2, 13), false);
					}
					xhorizD = getFilteredData();
					isObject = true;
					break;
				}
			}
			synchronized (this) {
				leftMotor.stop(true);
				rightMotor.stop(false);
			}

			if (isObject == true) {
				checkandgrab(85, xhorizD - 8);
				if (found == true) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					System.exit(0);
					//break;
				} else {
		

			//		odometer.setPosition(new double[] { 0.0, 0.0, 355.0 }, new boolean[] { false, false, true });

					nav.TravelTo(ySave, xSave + 30);
					Sound.beepSequence();

					nav.doNavigation(false);

					nav.turnTo(0);
				}

			} else {
				sensorMotor.rotateTo(90, false);
				synchronized (this) {
					leftMotor.stop(true);
					rightMotor.stop(false);
				}

			}

		}

		synchronized (this) {

			leftMotor.rotate(convertAngle(2.2, 15.3, 89.8), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, 89.8), false);
		}

		for (k = 0; k < (opzTR_Y - opzBL_Y) / 30; k++) {
			if (found) {
				Sound.beepSequenceUp();
				return;
			}
			xSave = odometer.getX();
			ySave = odometer.getY();
			leftMotor.setSpeed(100);
			rightMotor.setSpeed(100);
			sensorMotor.rotateTo(-90, false);
			isObject = false;
			synchronized (this) {
				leftMotor.forward();
				rightMotor.forward();
			}
			while (closeTo(odometer.getY(), ySave - 30) == false) {

				if (getFilteredData() < 20) {
					synchronized (this) {
						leftMotor.rotate(convertDistance(2.2, 13), true);
						rightMotor.rotate(convertDistance(2.2, 13), false);
					}
					xhorizD = getFilteredData();
					isObject = true;
					break;
				}
			}

			synchronized (this) {
				leftMotor.stop(true);
				rightMotor.stop(false);
			}

			if (isObject == true) {
				checkandgrab(85, xhorizD - 8);
				if (found == true) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					System.exit(0);
				//	break;
				} else {
			//		odometer.setPosition(new double[] { 0.0, 0.0, 280.0 }, new boolean[] { false, false, true });

					nav.TravelTo(xSave, ySave - 30);
					Sound.beepSequence();

					nav.doNavigation(false);
					nav.turnTo(270);
				}
			} else {
				sensorMotor.rotateTo(90, false);
				synchronized (this) {
					leftMotor.stop(true);
					rightMotor.stop(false);
				}

			}

		}

		synchronized (this) {

			leftMotor.rotate(convertAngle(2.2, 15.3, 89.8), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, 89.8), false);
		}
	}

	private boolean closeTo(double x, double y) {
		if (Math.abs(x - y) < 2) {
			return true;
		}
		return false;
	}
	/*
	 * private void horizFlagSearch(double Distance) {
	 * sensorMotor.rotateTo(0,false); synchronized(this) { leftMotor.stop(true);
	 * rightMotor.stop(false); leftMotor.rotate(convertAngle(2.2, 15, 45),
	 * true); rightMotor.rotate(-convertAngle(2.2, 15, 45), false); }
	 * synchronized(this) {
	 * leftMotor.rotate(convertDistance(2.2,Distance-5),true);
	 * rightMotor.rotate(convertDistance(2.2,Distance-5),false); }
	 * 
	 * leftMotor.stop(true); rightMotor.stop(false);
	 * 
	 * 
	 * }
	 * 
	 */

	/*
	 * private void flagSearch() { sensorMotor.setSpeed(30);
	 * leftMotor.setSpeed(100); rightMotor.setSpeed(100);
	 * sensorMotor.rotateTo(-90, true);
	 * 
	 * minDis=getFilteredData();
	 * 
	 * while(sensorMotor.isMoving()) { if(getFilteredData()<30 &&
	 * getFilteredData()<minDis) { minDis=getFilteredData(); tempDist=minDis;
	 * targetAng=sensorMotor.getPosition();
	 * 
	 * isObject=true; } } if(isObject==true) {
	 * checkandgrab(-targetAng-3,tempDist-3);
	 * 
	 * } else { sensorMotor.rotateTo(0); return; } }
	 */

	private void checkandgrab(double angle, double distance) {
		sensorMotor.rotateTo(0, false);
		synchronized (this) {
			leftMotor.rotate(convertAngle(2.2, 15.3, angle), true);
			rightMotor.rotate(-convertAngle(2.2, 15.3, angle), false);
		}

		synchronized (this) {
			leftMotor.rotate(convertDistance(2.2, distance), true);
			rightMotor.rotate(convertDistance(2.2, distance), false);
		}

		if (colour() == targetID) {
			synchronized (this) {
				leftMotor.rotate(-convertDistance(2.2, 5), true);
				rightMotor.rotate(-convertDistance(2.2, 5), false);
			}
			Sound.beep();
			synchronized (this) {
				leftMotor.rotate(convertAngle(2.2, 15.3, 180), true);
				rightMotor.rotate(-convertAngle(2.2, 15.3, 180), false);
			}
		
			synchronized (this) {
				leftMotor.rotate(-convertDistance(2.2, 5), true);
				rightMotor.rotate(-convertDistance(2.2, 5), false);
			}

			claw.rotate(-400);
			claw.stop();
			synchronized (this) {
				leftMotor.rotate(convertDistance(2.2, distance), true);
				rightMotor.rotate(convertDistance(2.2, distance), false);
			}
			Sound.beep();
			nav.turnTo(90);
			synchronized (this) {
				leftMotor.stop(true);
				rightMotor.stop(false);
			}
			found = true;
			return;
		} else {
			synchronized (this) {
				synchronized (this) {
					leftMotor.rotate(-convertDistance(2.2, 5), true);
					rightMotor.rotate(-convertDistance(2.2, 5), false);
				}
				synchronized (this) {
					leftMotor.rotate(convertAngle(2.2, 15.3, 180), true);
					rightMotor.rotate(-convertAngle(2.2, 15.3, 180), false);
				}
				synchronized (this) {
					leftMotor.rotate(-convertDistance(2.2, 5), true);
					rightMotor.rotate(-convertDistance(2.2, 5), false);
				}
				claw.rotate(-400);
				claw.stop();
				synchronized (this) {
					leftMotor.rotate(convertDistance(2.2, distance), true);
					rightMotor.rotate(convertDistance(2.2, distance), false);
				}
				Sound.buzz();
				synchronized (this) {
					leftMotor.rotate(convertAngle(2.2, 15.3, 90), true);
					rightMotor.rotate(-convertAngle(2.2, 15.3, 90), false);
				}
				synchronized (this) {
					leftMotor.stop(true);
					rightMotor.stop(false);
				}
				claw.rotate(400);
			}

		}

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

	private static int colour() {
		float[] sampleRGB = new float[10];
		colorSensor.getRGBMode().fetchSample(sampleRGB, 0);

		if ((sampleRGB[0] * 1000 < sampleRGB[1] * 1000) && (sampleRGB[1] * 1000 < sampleRGB[2] * 1000)) {
			return 1; // dark blue
		}
		if ((sampleRGB[0] * 1000 < sampleRGB[1] * 1000) && (sampleRGB[1] * 1000 > sampleRGB[2] * 1000)
				&& (((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) < 0.8)) {
			return 2; // light blue
		}
		if ((sampleRGB[0] * 1000 > sampleRGB[1] * 1000)
				&& ((int) ((sampleRGB[1] * 1000) / (sampleRGB[2] * 1000)) > 2)) {
			return 3; // yellow
		}
		if ((sampleRGB[0] * 1000 > sampleRGB[1] * 1000)
				&& ((int) ((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) > 1)) {
			return 4; // red
		}
		if ((Math.round((sampleRGB[0] * 1000) * 0.7 / (sampleRGB[2] * 1000)) == 1)
				&& (((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) < 1.1)
				&& (((sampleRGB[0] * 1000) / (sampleRGB[1] * 1000)) > 0.9)) {
			return 5; // white
		}
		return 0;
	}

	public static boolean isfound() {
		return found;
	}

}