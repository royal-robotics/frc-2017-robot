package org.usfirst.frc.team2522.robot;

import org.opencv.core.Mat;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
//import edu.wpi.cscore.MjpegServer;
//import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.UsbCamera;

//import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
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
	Spark unjammer = new Spark(4);

	CANTalon shooter1 = new CANTalon(0);
	CANTalon shooter2 = new CANTalon(1);
	CANTalon feeder = new CANTalon(2);
	CANTalon climber1 = new CANTalon(3);
	CANTalon climber2 = new CANTalon(4);
	CANTalon pickup = new CANTalon(5);
	
	Encoder leftDriveEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3));
	Encoder rightDriveEncoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
	Encoder shooterEncoder = new Encoder(new DigitalInput(0), new DigitalInput(1));
	
	RobotDrive myDrive = new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
	
	DoubleSolenoid shifter = new DoubleSolenoid(0, 3, 4);
	DoubleSolenoid intake = new DoubleSolenoid(0, 2, 5);
	DoubleSolenoid gearWall = new DoubleSolenoid(0, 1, 6);
	DoubleSolenoid gearPushout = new DoubleSolenoid(0, 0, 7);
	
	DoubleSolenoid gearDrapes = new DoubleSolenoid(1, 3, 4);
	DoubleSolenoid shooterHood = new DoubleSolenoid(1, 2, 5);
		
	AnalogInput sensor = new AnalogInput(0);
	AHRS gyro;
	
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	Joystick operatorStick = new Joystick(2);
	
	Button shiftButton = new Button(rightStick, 1, Button.ButtonType.Toggle);
	Button intakeButton = new Button(operatorStick, 1, Button.ButtonType.Toggle);
	Button gearWallButton = new Button(operatorStick, 2, Button.ButtonType.Toggle);
	Button gearPushoutButton = new Button(operatorStick, 3, Button.ButtonType.Toggle);
	Button gearDrapesButton = new Button(operatorStick, 4, Button.ButtonType.Toggle);
	Button shooterHoodButton = new Button(operatorStick, 5, Button.ButtonType.Toggle);
	
	Button shooterButton = new Button(operatorStick, 6, Button.ButtonType.Hold);
	Button pickupButton = new Button(operatorStick, 7, Button.ButtonType.Hold);
	Button climberButton = new Button(operatorStick, 8, Button.ButtonType.Hold);
	Button feederButton = new Button(operatorStick, 9, Button.ButtonType.Hold);
	Button unjammerButton = new Button(operatorStick, 10, Button.ButtonType.Hold);

	//  (<Wheel Diameter in Inches> * <Pi>) / (<Pulses Per Rotation> * <Encoder Mount Gearing> <Third Stage Gearing>)  //
	public static double driveTranDistancePerPulse = (3.50 * 3.1415) / (360.00);
	public static double shooterDistancePerPulse = (3.50 * 3.1415) / (1.00 * 1.00) * (1.00);

	public void robotInit()
	{
		leftDrive1.setSafetyEnabled(false);
		leftDrive2.setSafetyEnabled(false);
		rightDrive1.setSafetyEnabled(false);
		rightDrive2.setSafetyEnabled(false);
		SmartDashboard.putBoolean("wheelDrive", false);

		 try {
				/***********************************************************************
				 * navX-MXP:
				 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
				 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * navX-Micro:
				 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
				 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * Multiple navX-model devices on a single robot are supported.
				 ************************************************************************/
	            gyro = new AHRS(SPI.Port.kMXP);
	        } catch (RuntimeException ex ) {
	        }
		 gyro.reset();
		leftDriveEncoder.setReverseDirection(true);
		leftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
		leftDriveEncoder.reset();
		rightDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
		rightDriveEncoder.reset();
    	shooterEncoder.setDistancePerPulse(shooterDistancePerPulse);
    	shooterEncoder.reset();
    	
    	shifter.set(DoubleSolenoid.Value.kForward);
    	intake.set(DoubleSolenoid.Value.kReverse);
    	gearWall.set(DoubleSolenoid.Value.kReverse);
    	gearPushout.set(DoubleSolenoid.Value.kReverse);
    	gearDrapes.set(DoubleSolenoid.Value.kReverse);
    	shooterHood.set(DoubleSolenoid.Value.kForward);
		
    	visionThread = new Thread(() -> 
    	{
    		UsbCamera camera = new UsbCamera("camera", 0);
			UsbCamera camera2 = new UsbCamera("camera2", 1);
    		CvSink Cvcamera = new CvSink("cam0");
    		CvSink Cvcamera2 = new CvSink("cam1");
    		Mat mat = new Mat();
    		Cvcamera.grabFrame(mat);
    		mat.get(0, 0);
    		Cvcamera2.grabFrame(mat);
    		mat.get(0, 0);
			CameraServer.getInstance().addCamera(camera);
			MjpegServer server = CameraServer.getInstance().addServer("camera");
			server.setSource(camera);
			
			
			while (!Thread.interrupted()) {
				if (rightStick.getRawButton(2) && (!cameraButtonHeld))
				{
					cameraButtonHeld = true;
					if (camera1on)
					{
						server.setSource(camera2);
						camera1on = false;
					}
					else
					{
						server.setSource(camera);
						camera1on = true;
					}
				}
				else if (!rightStick.getRawButton(2) && (cameraButtonHeld))
				{
					cameraButtonHeld = false;
				}
				
			}
			});    	
    	//CameraServer.getInstance().startAutomaticCapture();

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
	
	public void teleopPeriodic()
	{
		if (!wheelDrive)
		{
			myDrive.tankDrive(leftStick, rightStick, true);
		}
		else
		{
			double speed = rightStick.getRawAxis(1);
			double direction = leftStick.getRawAxis(0);
			if (speed > -0.2 && speed < 0.2)
			{
				speed = 0.0;
			}
				if (direction < -0.2)
				{
					rightDrive1.set(-speed);
					rightDrive2.set(-speed);
					leftDrive1.set(0.0);
					leftDrive2.set(0.0);
					if (leftStick.getRawButton(5))
					{
						leftDrive1.set(-speed);
						leftDrive2.set(-speed);
					}
					else
					{
						leftDrive1.set(0.0);
						leftDrive2.set(0.0);
					}
				}
				else if (direction > 0.2)
				{
					leftDrive1.set(speed);
					leftDrive2.set(speed);
					if (leftStick.getRawButton(5))
					{
						rightDrive1.set(speed);
						rightDrive2.set(speed);
					}
					else
					{
						rightDrive1.set(0.0);
						rightDrive2.set(0.0);
					}
				}
				else
				{
					rightDrive1.set(-speed);
					rightDrive2.set(-speed);
					leftDrive1.set(speed);
					leftDrive2.set(speed);
				}
		}
	
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

		if (shooterButton.isPressed())
		{
			shooter1.set(1.0);
			shooter2.set(-1.0);
		}
		else
		{
			shooter1.set(0.0);
			shooter2.set(0.0);
		}
		
		if (pickupButton.isPressed())
		{
			pickup.set(1.0);
		}
		else{
			pickup.set(0.0);
		}
		
		if (climberButton.isPressed())
		{
			climber1.set(1.0);
			climber2.set(1.0);
		}
		else
		{
			climber1.set(0.0);
			climber2.set(0.0);
		}
		
		if (feederButton.isPressed())
		{
			feeder.set(1.0);
		}
		else
		{
			feeder.set(0.0);
		}
		
		if (unjammerButton.isPressed())
		{
			unjammer.set(1.0);
		}
		else
		{
			unjammer.set(0.0);
		}
		
		ButtonFeeder.INSTANCE.feed();
		
		writeDashboard();
		
		
	}
	
	boolean wheelDrive = false;
	
	public void writeDashboard()
	{
		SmartDashboard.putNumber("photoelectricSensor", sensor.getVoltage());
		SmartDashboard.putNumber("gyro", gyro.getAngle());
		SmartDashboard.putNumber("leftDriveEncoder", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("rightDriveEncoder", rightDriveEncoder.getDistance());
		SmartDashboard.putNumber("shooterEncoder", shooterEncoder.getRate());
		wheelDrive = SmartDashboard.getBoolean("wheelDrive", false);
	}
	
}
