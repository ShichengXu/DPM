package finalproject;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Localization {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	public static int ROTATION_SPEED = 100;
	public double theta;
	private static double leftR = 2.1, rightR = 2.1, width = 15, thetaTurn;
	private static boolean MoveFW=true;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odo;
	private SampleProvider usSensor;
	private SampleProvider touch1,touch2;
	private float[] usData;
	private float[] touchSample1,touchSample2;
	private LocalizationType locType;

	public Localization(Odometer odo, SampleProvider usSensor, float[] usData, SampleProvider touch1, SampleProvider touch2,float[] touchSample1,float[] touchSample2,
			LocalizationType locType, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.touchSample1=touchSample1;
		this.touchSample2=touchSample2;
		this.locType = locType;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.touch1=touch1;
		this.touch2=touch2;

	}

	public void doLocalization() {
		double angleA, angleB;

		if (locType == LocalizationType.FALLING_EDGE) {              //falling edge
			  
			while (getFilteredData() <= 45) {                        //if the robot starts facing the wall, rotate clock-wise
				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.forward();
				rightMotor.backward();
			}

			while (getFilteredData() > 30) {                         //if the robot start facing against the wall
				leftMotor.setSpeed(ROTATION_SPEED);                  //rotates clock-wise
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.forward();
				rightMotor.backward();
			}
			angleA = odo.getAng();                                   //usSensor detects the first wall. Record angle
			Sound.buzz();

			while (getFilteredData() < 30) {                         //let the robot rotates counter-clock wise 

				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.backward();
				rightMotor.forward();
			}

			while (getFilteredData() > 30) {

				leftMotor.setSpeed(ROTATION_SPEED);
				rightMotor.setSpeed(ROTATION_SPEED);
				leftMotor.backward();
				rightMotor.forward();                                //until it detects the second wall. Record the angle.
			}
			angleB = odo.getAng();
			Sound.beep();
			leftMotor.stop(true);
			rightMotor.stop(false);
 
			if (angleA < angleB) {                                   //calculate the angle the robot needs to turn to be straight.
				thetaTurn = -(angleB - angleA - 70) / 2;              
			}
			if (angleA > angleB) {
				thetaTurn = -(angleB + 145 - (angleA + angleB) / 2);
			}

			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			
			leftMotor.rotate(-convertAngle(leftR, width,thetaTurn), true);
			rightMotor.rotate(convertAngle(rightR, width, thetaTurn), false);

			odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {   //update the odometer to 0,0,0 (to make our second part easier )
					true, true, true });
			
			
			
			turnAngle(90);
			while(MoveFW==true)
			{
				touch1.fetchSample(touchSample1,0);
				touch2.fetchSample(touchSample2,0);
				
				leftMotor.setSpeed(120);
				rightMotor.setSpeed(120);
				leftMotor.forward();
				rightMotor.forward();
				if (touchSample1[0]==1 && touchSample2[0]==1)
				{
					leftMotor.stop(true);
					rightMotor.stop(false);
					break;
				}
			}
			
		//	touch.fetchSample(touchSample,0);
			
			try {

				Thread.sleep(2000L); // one second

			}

			catch (Exception e) {

			}
		}

		}

	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		if (distance > 50) {
			distance = 50;
		}

		return distance;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
    
   public static void turnAngle(double Angle)
   {
	   leftMotor.setSpeed(100);
	   rightMotor.setSpeed(100);
	   leftMotor.rotate(-convertAngle(leftR, width, Angle), true);
	   rightMotor.rotate(convertAngle(rightR, width, Angle), false);
	   
   }

}