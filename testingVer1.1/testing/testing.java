package testing;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
public class testing {
	
	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	
	// Constants
	public static final double RWHEEL_RADIUS = 2.2;
	public static final double LWHEEL_RADIUS = 2.2;
	
	public static final double TRACK = 15;

	public static void main(String[] args) {
		int buttonChoice;

		// some objects that need to be instantiated
		
		final TextLCD t = LocalEV3.get().getTextLCD();
		
		
        
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString(" run   | square  ", 0, 2);
			t.drawString("straight  |    ", 0, 3);
			t.drawString("     | driver ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			Odometer odo = new Odometer(leftMotor, rightMotor,30,true);
			OdometerDisplay odometryDisplay = new OdometerDisplay(odo);
			leftMotor.setSpeed(300);
			rightMotor.setSpeed(300);
			leftMotor.forward();
			rightMotor.forward();
	
			
		} else {
			// start the odometer, the odometry display and (possibly) the
			// odometry correction
			
			Odometer odo = new Odometer(leftMotor, rightMotor,30,true);
			OdometerDisplay odometryDisplay = new OdometerDisplay(odo);

			// spawn a new Thread to avoid SquareDriver.drive() from blocking
			(new Thread() {
				public void run() {
					SquareDriver.drive(leftMotor, rightMotor, LWHEEL_RADIUS, RWHEEL_RADIUS, TRACK);
				}
			}).start();
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}