package finalproject;

import java.util.ArrayList;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class searching {
	private static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	private static final EV3MediumRegulatedMotor claw = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private static EV3LargeRegulatedMotor leftMotor, rightMotor,sensorMotor;
	Navigation nav;
	private Odometer odometer;
	private SampleProvider usSensor;
	private float[] usData;
	private SampleProvider touch1,touch2;
	private float[] touchSample1,touchSample2;
	private double distance;
	private int filterControl;
	private double proportion=20;	
	private double perror;
	private double amperror;
	private int cruisingSpeed = 200, FILTER_OUT = 80,motorStraight=100;
	private static final int maxSpeed=400;
	private int bandCenter=15;
	static boolean found;
	private double temp=100,targetAng;
//	ArrayList <Double> al = new ArrayList<Double>();
	

public searching(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor,EV3LargeRegulatedMotor sensorMotor,Odometer odometer,SampleProvider usSensor, float[] usData,SampleProvider touch1, SampleProvider touch2,
		float[] touchSample1,float[] touchSample2,Navigation nav)
{
	this.nav = nav;
	this.leftMotor=leftMotor;
	this.rightMotor=rightMotor;
	this.sensorMotor=sensorMotor;
	this.odometer=odometer;
	this.usSensor = usSensor;
	this.usData = usData;
	this.touchSample1=touchSample1;
	this.touchSample2=touchSample2;
	this.touch1=touch1;
	this.touch2=touch2;
	found=false;
}
public void doSearching() {
	leftMotor.setSpeed(cruisingSpeed);
	rightMotor.setSpeed(cruisingSpeed);
	nav.turnTo(0);
	while(odometer.getAng()<90)
	{
		leftMotor.backward();
		rightMotor.forward();
		temp=getFilteredData();
		if(getFilteredData()<temp)
		{
			targetAng=odometer.getAng();
		}
		
	}
	nav.turnTo(targetAng);
	

	while(getFilteredData()>10)
	{
	
		leftMotor.forward();
		rightMotor.forward();
	}
	leftMotor.stop(true);
	rightMotor.stop(false);
	leftMotor.rotate(convertAngle(2.2, 15, 89.8), true);
	rightMotor.rotate(-convertAngle(2.2, 15, 89.8), false);
	claw.rotate(-270);
	sensorMotor.rotate(-90);
	
	while(!(colour()==1))
	{
	if (getFilteredData() >100 && filterControl < FILTER_OUT) {
		// bad value, do not set the distance var, however do increment the filter value
		filterControl ++;
	} else if (getFilteredData() > 100){
		// set distance to this.distance
		distance=100;
	} else {
		// distance went below 100, therefore reset everything.
		filterControl = 0;
		distance = getFilteredData();
	}

	perror = distance-bandCenter;
	amperror= perror*proportion;
	if(distance>60){
		// if distance >80, the robot is about to do a U-turn
		leftMotor.setSpeed(maxSpeed); 
		rightMotor.setSpeed(motorStraight); 
		//right wheel will speed up in order to turn left
		leftMotor.forward();
		rightMotor.forward();
	}

	else 
	{
		//general proportion error correction
		rightMotor.setSpeed(motorStraight);
		if((motorStraight+amperror)>maxSpeed)
		{
			leftMotor.setSpeed(maxSpeed);
		}
		//left wheel remains constant speed
		else{
		leftMotor.setSpeed((int) (motorStraight+amperror));
		}
		//right wheel will continuously correcting its speed according to errors. 
		leftMotor.forward();
		rightMotor.forward();
	}
	}

	claw.rotate(-40);
	claw.stop();

}

private float getFilteredData() {
	usSensor.fetchSample(usData, 0);
	float distance = usData[0] * 100;
	if (distance > 100) {
		distance = 100;
	}
	return distance;
}


private boolean collide()
{
	touch1.fetchSample(touchSample1,0);
	touch2.fetchSample(touchSample2,0);
	if(touchSample1[0]==1||touchSample2[0]==1)
	{
		return true;
	}
	return false;
}

private static int convertAngle(double radius, double width, double angle) {
	return convertDistance(radius, Math.PI * width * angle / 360.0);
}

private static int convertDistance(double radius, double distance) {
	return (int) ((180.0 * distance) / (Math.PI * radius));
}

private static int colour(){
	float[] sampleRGB = new float[10];
	colorSensor.getRGBMode().fetchSample(sampleRGB,0);
	
	
	if((sampleRGB[0]*1000 < sampleRGB[1]*1000)&&(sampleRGB[1]*1000 < sampleRGB[2]*1000)){
		return 1;  //dark blue
	}
	if((sampleRGB[0]*1000 < sampleRGB[1]*1000)&&(sampleRGB[1]*1000 > sampleRGB[2]*1000)&&(((sampleRGB[0]*1000)/(sampleRGB[1]*1000))<0.8)){
		return 2;  //light blue
	}
	if((sampleRGB[0]*1000 > sampleRGB[1]*1000)&&((int)((sampleRGB[1]*1000)/(sampleRGB[2]*1000))>2)){
		return 3;  //yellow
	}
	if((sampleRGB[0]*1000 > sampleRGB[1]*1000)&&((int)((sampleRGB[0]*1000)/(sampleRGB[1]*1000))>1)){
		return 4;  //red
	}
	if((Math.round((sampleRGB[0]*1000)*0.7/(sampleRGB[2]*1000))==1)&&(((sampleRGB[0]*1000)/(sampleRGB[1]*1000))<1.1)&&(((sampleRGB[0]*1000)/(sampleRGB[1]*1000))>0.9)){
		return 5;  //white
	}
	return 0;
}

public static boolean isfound() {
	return found;
}



}