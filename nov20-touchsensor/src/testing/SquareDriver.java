package testing;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.SampleProvider;

public class SquareDriver {
	private static final int FORWARD_SPEED = 400;
	private static final int ROTATE_SPEED = 200;
	
	private static final Port leftTouchPort= LocalEV3.get().getPort("S3");
	private static final Port rightTouchPort=LocalEV3.get().getPort("S4");
	static SampleProvider touch1 = new EV3TouchSensor(leftTouchPort);
	static SampleProvider touch2 = new EV3TouchSensor(rightTouchPort);
	
	static float[] touchSample1=new float[touch1.sampleSize()];
	static float[] touchSample2=new float[touch2.sampleSize()];
	private static boolean touch=false;
	
	
	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double width) {
		int buttonChoice;
	
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	while(true)
	{
		while(touch==false)
		{
         
		    if(collide()==true)
		{
			Sound.beep();
			leftMotor.stop(true);
			rightMotor.stop(false);
			touch=true;
		}
		   }
		    
		    while(touch==true)
		    {	
		    buttonChoice = Button.waitForAnyPress();
			if(buttonChoice==Button.ID_LEFT)
			{
		    leftMotor.forward();
			rightMotor.forward();
			touch=false;
			}
		
		}
	

	}
	}
	
	private static boolean collide()
	{
		touch1.fetchSample(touchSample1,0);
		touch2.fetchSample(touchSample2,0);
		if(touchSample1[0]==1||touchSample2[0]==1)
		{
			return true;
		}
		return false;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}