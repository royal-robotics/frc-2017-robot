package org.usfirst.frc.team2522.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
//import edu.wpi.cscore.MjpegServer;
//import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.UsbCamera;

//import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

	//  drive system test  //
public class Robot extends IterativeRobot
{
	Thread visionThread;
	boolean cameraButtonHeld = false;
	boolean camera1on = false;
	
	VictorSP leftDrive1 = new VictorSP(0);
	VictorSP leftDrive2 = new VictorSP(1);
	VictorSP rightDrive1 = new VictorSP(2);
	VictorSP rightDrive2 = new VictorSP(3);
	
	CANTalon pickup = new CANTalon(5);
	CANTalon feeder = new CANTalon(2);
	CANTalon shooter1 = new CANTalon(0);
	CANTalon shooter2 = new CANTalon(1);
	
	CANTalon climber1 = new CANTalon(3);
	CANTalon climber2 = new CANTalon(4);
	
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	Joystick operatorStick = new Joystick(2);
	
	Encoder leftDriveEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3));
	Encoder rightDriveEncoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
	
	Encoder shooterEncoder = new Encoder(new DigitalInput(6), new DigitalInput(7));
	
	RobotDrive myDrive;
	
	DoubleSolenoid shifter = new DoubleSolenoid(0, 3, 4);
	
	DoubleSolenoid intake = new DoubleSolenoid(0, 2, 5);
	DoubleSolenoid gearWall = new DoubleSolenoid(0, 1, 6);
	DoubleSolenoid gearPushout = new DoubleSolenoid(0, 0, 7);
	DoubleSolenoid gearDrapes = new DoubleSolenoid(1, 3, 4);
	DoubleSolenoid shooterHood = new DoubleSolenoid(1, 2, 5);
		
	AnalogInput sensor = new AnalogInput(0);
	
	// Joysticks //
	Joystick leftstick = new Joystick(0);
	Joystick rightstick = new Joystick(1);
	Joystick operatorstick = new Joystick(2);
	
	Button shiftButton = new Button(rightstick, 1, Button.ButtonType.Toggle);
	Button intakeButton = new Button(rightstick, 2, Button.ButtonType.Toggle);
	Button gearWallButton = new Button(rightstick, 3, Button.ButtonType.Toggle);
	Button gearPushoutButton = new Button(rightstick, 4, Button.ButtonType.Toggle);
	Button gearDrapesButton = new Button(rightstick, 5, Button.ButtonType.Toggle);
	Button shooterHoodButton = new Button(rightstick, 6, Button.ButtonType.Toggle);

	//  (<Wheel Diameter in Inches> * <Pi>) / (<Pulses Per Rotation> * <Encoder Mount Gearing> <Third Stage Gearing>)  //
//	public static double driveTranDistancePerPulse = ( * 3.1415) / ( * ( / ) * ( / ));
	
	public void robotInit()
	{
//      frontLeftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
//      frontLeftDriveEncoder.reset();
//		frontRightDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
//		frontRightDriveEncoder.reset();
		
    	myDrive = new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
    	shifter.set(DoubleSolenoid.Value.kForward);
    	intake.set(DoubleSolenoid.Value.kForward);
    	gearWall.set(DoubleSolenoid.Value.kForward);
    	gearPushout.set(DoubleSolenoid.Value.kForward);
    	gearDrapes.set(DoubleSolenoid.Value.kForward);
    	shooterHood.set(DoubleSolenoid.Value.kForward);

//    	shooterEncoder.setDistancePerPulse(shooterDistancePerPulse);
//    	shooterEncoder.reset();
    	
    	visionThread = new Thread(() -> 
    	{
    		UsbCamera camera1 = new UsbCamera("camera1", 0);
    		//UsbCamera camera2 = new UsbCamera("camera2", 1);
    		
    		//CameraServer.getInstance().addCamera(camera1);
    		//CameraServer.getInstance().addCamera(camera2);
    		
    		while (!Thread.interrupted()) {
    		
    		}
		});

		visionThread.setDaemon(true);
		visionThread.start();
	}

	public void autonomousInit()
	{
		
	}

	public void autonomousPeriodic()
	{
		
	}

	public void teleopInit()
	{

	}
	
	boolean shifted = false;
	boolean intakeOut = false;
	boolean gearWallUp = false;
	boolean gearPushin = false;
	boolean gearDraped = false;
	boolean shooterHoodUp = false;
	boolean LEDOn = false;
	
	public void teleopPeriodic()
	{
		myDrive.tankDrive(leftStick, rightStick, true);
		
		if (shiftButton.isPressed()) //Toggle button
		{
			if (shifted)
			{
				shifter.set(DoubleSolenoid.Value.kReverse);
				shifted = false;
			}
			else
			{
				shifter.set(DoubleSolenoid.Value.kForward);
				shifted = true;
			}
		}
		
		if (intakeButton.isPressed())
		{	
			if (intakeOut)
			{
				intake.set(DoubleSolenoid.Value.kReverse);
				intakeOut = false;
			}
			else
			{
				intake.set(DoubleSolenoid.Value.kForward);
				intakeOut = true;
			}
		}
		
		if (gearWallButton.isPressed())
		{	
			if (gearWallUp)
			{
				gearWall.set(DoubleSolenoid.Value.kReverse);
				gearWallUp = false;
			}
			else
			{
				gearWall.set(DoubleSolenoid.Value.kForward);
				gearWallUp = true;
			}
		}

		if (gearPushoutButton.isPressed())
		{	
			if (gearPushin)
			{
				gearPushout.set(DoubleSolenoid.Value.kReverse);
				gearPushin = false;
			}
			else
			{
				gearPushout.set(DoubleSolenoid.Value.kForward);
				gearPushin = true;
			}
		}

		if (gearDrapesButton.isPressed())
		{	
			if (gearDraped)
			{
				gearDrapes.set(DoubleSolenoid.Value.kReverse);
				gearDraped = false;
			}
			else
			{
				gearDrapes.set(DoubleSolenoid.Value.kForward);
				gearDraped = true;
			}
		}

		if (shooterHoodButton.isPressed())
		{	
			if (shooterHoodUp)
			{
				shooterHood.set(DoubleSolenoid.Value.kReverse);
				shooterHoodUp = false;
			}
			else
			{
				shooterHood.set(DoubleSolenoid.Value.kForward);
				shooterHoodUp = true;
			}
		}

		if (leftStick.getRawButton(2))
		{
			shooter1.set(1.0);
			shooter2.set(1.0);
		}
		else
		{
			shooter1.set(0.0);
			shooter2.set(0.0);
		}
		
		if (leftStick.getRawButton(3))
		{
			pickup.set(1.0);
		}
		else{
			pickup.set(0.0);
		}
		
		if (leftStick.getRawButton(4))
		{
			climber1.set(1.0);
			climber2.set(1.0);
		}
		else
		{
			climber1.set(0.0);
			climber2.set(0.0);
		}
		
		if (leftStick.getRawButton(5))
		{
			feeder.set(1.0);
		}
		else
		{
			feeder.set(0.0);
		}
		
		
		SmartDashboard.putNumber("photoelectricSensor", sensor.getVoltage());
		
		ButtonFeeder.INSTANCE.feed();
	}
	
}
