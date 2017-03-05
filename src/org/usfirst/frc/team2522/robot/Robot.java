package org.usfirst.frc.team2522.robot;

import java.io.*;
import java.nio.ByteBuffer;
import java.sql.DriverPropertyInfo;

import org.opencv.core.Mat;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.hal.I2CJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

	//  drive system test  //
public class Robot extends IterativeRobot
{
	public Timer robotTimer = new Timer();
	private double lastTime = 0.0;
	private double lastBearing = 0.0;
	private double lastDistance = 0.0;
	private double lastVelocity = 0.0;
	private double lastAcceleration = 0.0;
	private double driveStraightBearing = 0.0;
	private double driveToStartDistance = 0.0;
	
	Thread visionThread;
	double currentExposure = -99.0;		// keep track of exposure changes so we do not hammer the USB camera with calls
	double currentWhiteBalance = -99.0;	// keep track of white balance changes so we do not hammer the USB camera with calls
	
	
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
	CANTalon intakeTalon = new CANTalon(5);
	
	Encoder leftDriveEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3));
	Encoder rightDriveEncoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
	
	RobotDrive myDrive = new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
	
	DoubleSolenoid shifter = new DoubleSolenoid(0, 3, 4);
	DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0, 2, 5);
	DoubleSolenoid gearWall = new DoubleSolenoid(0, 1, 6);
	DoubleSolenoid gearPushout = new DoubleSolenoid(0, 0, 7);	
	DoubleSolenoid gearDrapes = new DoubleSolenoid(1, 3, 4);
	DoubleSolenoid shooterHood = new DoubleSolenoid(1, 2, 5);
		
	AnalogInput sensor = new AnalogInput(0);
	AHRS gyro;
	
	//I2C i2cIOExpand = new I2C(I2C.Port.kOnboard, 0b0100000);
	//I2C i2cIOExpand = new I2C(I2C.Port.kOnboard, 32);//32, 16, 64
	I2C i2cIOExpand = new I2C(I2C.Port.kOnboard, 0x02);//0x20, 0x02
	
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	Joystick operatorStick = new Joystick(2);
	
	// Driver Controls
	//
	Button shiftButton = new Button(leftStick, 1, ButtonType.Toggle);
	Button cameraLowButton = new Button(leftStick, 2, ButtonType.Hold);
	Button driveStraightButton = new Button(leftStick, 3, ButtonType.Hold);
	
	// Debug Buttons
	Button i2cButton = new Button(leftStick, 11, ButtonType.Toggle);
	Button motionRecordButton = new Button(leftStick, 7, ButtonType.Hold);	

	Button quickTurnButtonLeft = new Button(rightStick, 5, ButtonType.Hold);
	Button quickTurnButtonRight = new Button(rightStick, 6, ButtonType.Hold);
	Button driverIntakeButton = new Button(rightStick, 2, ButtonType.Toggle);
	
	// Operator Controls
	//
	POVButton intakeButton = new POVButton(operatorStick, 0, 0, ButtonType.Toggle);
	POVButton gearPushoutButton = new POVButton(operatorStick,0, 90, ButtonType.Hold);
	
	Button gearDrapesButton = new Button(operatorStick, 1, ButtonType.Hold);
	Button gearWallDnButton = new Button(operatorStick, 2, ButtonType.Toggle);
	Button gearWallUpButton = new Button(operatorStick, 4, ButtonType.Toggle);

	Button shooterHoodButton = new Button(operatorStick, 5, ButtonType.Toggle);
	Button feederButton = new Button(operatorStick, 6, ButtonType.Hold);
	Button intakeRollerButton = new Button(operatorStick, 7, ButtonType.Hold);	
	Button shooterButton = new Button(operatorStick, 8, ButtonType.Hold);
	
	Button climberButton = new Button(operatorStick, 9, ButtonType.Hold);
	Button unjammerButton = new Button(operatorStick, 10, ButtonType.Hold);
	
	
	//  (<Wheel Diameter in Inches> * <Pi>) / (<Pulses Per Rotation> * <Encoder Mount Gearing> <Third Stage Gearing>)  //
	public static double driveTranDistancePerPulse = (3.50 * 3.1415) / (360.00);
	public static double shooterDistancePerPulse = (3.50 * 3.1415) / (1.00 * 1.00) * (1.00);

	boolean wheelDrive = true;
	
	PrintStream ps = null;
	Timer motionProfileTimer = new Timer();
	
	CameraServer cameraServer = CameraServer.getInstance();
	CvSource  cameraStream = CameraServer.getInstance().putVideo("camera", 640, 480);
	UsbCamera cameraHigh = null;
	UsbCamera cameraLow = null;
	CvSink cameraHighSink = null;
	CvSink cameraLowSink = null;
//	MjpegServer server = CameraServer.getInstance().addServer("camera");
//	Mat mat = new Mat();
		
	public void robotInit()
	{		
		// Init Robot Timer
		//
		robotTimer.reset();
		robotTimer.start();
		lastTime = robotTimer.get();

		// Init Drive Base Encoders
		//
		leftDriveEncoder.setReverseDirection(true);
		leftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
		leftDriveEncoder.reset();
		rightDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
		rightDriveEncoder.reset();
				
		
		myDrive.setSafetyEnabled(false);
		leftDrive1.setSafetyEnabled(false);
		leftDrive2.setSafetyEnabled(false);
		rightDrive1.setSafetyEnabled(false);
		rightDrive2.setSafetyEnabled(false);
		
		intakeTalon.setSafetyEnabled(false);
		feeder.setSafetyEnabled(false);
		shooter1.setSafetyEnabled(false);
		shooter2.setSafetyEnabled(false);
		
		climber1.setSafetyEnabled(false);
		climber2.setSafetyEnabled(false);
		
		unjammer.setSafetyEnabled(false);
		
		wheelDrive = true;
		SmartDashboard.putBoolean("wheelDrive", true);

		try 
		{
			gyro = new AHRS(SPI.Port.kMXP);
			gyro.reset();
		}
		catch (RuntimeException ex )
		{			
		}
		
		shooter2.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooter2.configEncoderCodesPerRev(1024);//This is wrong? shooterDistancePerPulse?
		climber2.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		climber2.configEncoderCodesPerRev(1024);//This is wrong? shooterDistancePerPulse?
    	
    	shifter.set(DoubleSolenoid.Value.kForward);
    	intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    	gearWall.set(DoubleSolenoid.Value.kReverse);
    	gearPushout.set(DoubleSolenoid.Value.kForward);
    	gearDrapes.set(DoubleSolenoid.Value.kReverse);
    	shooterHood.set(DoubleSolenoid.Value.kReverse);
//    	visionThread = new Thread(() -> {
////    		UsbCamera camera = new UsbCamera("cam0", 0);
////    		MjpegServer server = CameraServer.getInstance().addServer("cam0");
////    		server.setSource(camera);
////    		CameraServer.getInstance().startAutomaticCapture(camera);
//    		while (!Thread.interrupted()) {
//    		}
//    	});

		cameraHigh = new UsbCamera("cam0", 0);
		if (cameraHigh != null) cameraHigh.setResolution(640, 480);
    	
//	   	cameraLow = new UsbCamera("cam1", 1);
    	if (cameraLow != null) cameraLow.setResolution(640, 480);
    	
    	if (cameraHigh != null)
    	{
    		cameraServer.addCamera(cameraHigh);
    		cameraServer.startAutomaticCapture(cameraHigh);
    	}
    	    	
//    	UsbCamera camera = new UsbCamera("cam2", 0);
//    	visionThread.setDaemon(true);
//		visionThread.start();
    	//CameraServer.getInstance().startAutomaticCapture(cameraHigh);
    	
//    	Cvcamera.grabFrame(mat);
//		mat.get(0, 0);
//		Cvcamera2.grabFrame(mat);
//		mat.get(0, 0);
		//CameraServer.getInstance().addCamera(camera);
		//CameraServer.getInstance().
//		server.setSource(camera);

		// Init Motion Control Values
		//
    	lastBearing = this.getCurrentBearing();    	
		driveStraightBearing = lastBearing;
    	
		lastDistance = leftDriveEncoder.getDistance();
		driveToStartDistance = lastDistance;
		
		lastVelocity = 0;
		lastAcceleration = 0;
	}

	/**
	 * 
	 */
	public void robotPeriodic()
	{
		double currentTime = robotTimer.get();
		double currentDistance = leftDriveEncoder.getDistance();
		double currentBearing = this.getCurrentBearing();
		
		double t = currentTime - lastTime;
		double d = currentDistance - lastDistance;
		double v = d / t;
		
		this.lastAcceleration = (v - lastVelocity) / t;		
		this.lastVelocity = v;
		this.lastDistance = currentDistance;
		this.lastBearing = currentBearing;
		this.lastTime = currentTime;
		
		writeDashboard();		
	}

	/**
	 * 
	 */
	public void autonomousInit()
	{
    	lastBearing = this.getCurrentBearing();    	
		driveStraightBearing = lastBearing;
		driveToStartDistance = this.getDistance();
	}

	/**
	 * 
	 */
	public void autonomousPeriodic()
	{
	
	}
	
	/**
	 * 
	 */
	public void disabledInit()
	{
		
	}
	
	/**
	 * 
	 */
	public void disabledPeriodic()
	{
	
	}

	/**
	 * 
	 */
	public void teleopInit()
	{
    	lastBearing = this.getCurrentBearing();    	
		driveStraightBearing = lastBearing;
		driveToStartDistance = this.getDistance();
	}
	
	/**
	 * 
	 */
	public void teleopPeriodic()
	{
		if (driveStraightButton.isPressed())
		{
			AutonomousController.driveTo(this, 0/*driveStraightBearing*/, 60.0/*driveToStartDistance + 60.0*/);
		}
		else
		{
			driveToStartDistance = this.getDistance();
			if (!wheelDrive)
			{
				myDrive.tankDrive(leftStick, rightStick, true);
			}
			else
			{
				double leftPower = leftStick.getY();			
				double rightPower = leftPower;
				double wheelValue = rightStick.getX();
				
				if (quickTurnButtonLeft.isPressed() || quickTurnButtonRight.isPressed())
				{
					rightPower = wheelValue;
					leftPower = -wheelValue;
					myDrive.tankDrive(leftPower, rightPower);
					driveStraightBearing = this.getBearing();
				}
				else
				{
					if (wheelValue < -0.05) 
					{
						leftPower = leftPower * (1+wheelValue);
						myDrive.tankDrive(leftPower, rightPower);
						driveStraightBearing = this.getBearing();
					}
					else if (wheelValue > 0.05)
					{
						rightPower = rightPower * (1 - wheelValue);
						myDrive.tankDrive(leftPower, rightPower);
						driveStraightBearing = this.getBearing();
					}
					else
					{
						AutonomousController.driveForward(this, driveStraightBearing, -leftPower);
					}
				}
			}
		}
		
		if (cameraLowButton.isPressed())
		{
		
		}
		else
		{

		}
		
		if (motionRecordButton.isPressed())
		{
			if (ps == null)
			{
				File f = new File("/home/lvuser/MotionProfile0.txt");
				for(int i = 0;f.exists();i++)
				{
					f = new File("/home/lvuser/MotionProfile" + i + ".txt");
				}
				try 
				{
					ps = new PrintStream(f);
				}
				catch(IOException e)
				{
					ps = null;
					e.printStackTrace();
				}
				
				System.out.println("MotionRecording started: " + f.getName());
				motionProfileTimer.reset();
				motionProfileTimer.start();
				
				gyro.reset();
				leftDriveEncoder.reset();
			}
			
			ps.println(String.valueOf(motionProfileTimer.get()) + "," + 
					   String.valueOf(leftDrive1.get()) + "," +
					   String.valueOf(this.getBearing()) + "," +
					   String.valueOf(this.getDistance()) + "," +
					   String.valueOf(this.getVelocity()) + "," +
					   String.valueOf(this.getAcceleration())
			);
		}
		else
		{
			if (ps != null)
			{
				ps.close();
				ps = null;
				motionProfileTimer.stop();
				motionProfileTimer.reset();
			}
		}
	
		if (shiftButton.isPressed()) //Toggle button
		{
			if (shifter.get() == DoubleSolenoid.Value.kForward)
			{
				shifter.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				shifter.set(DoubleSolenoid.Value.kForward);
			}
		}
		
		if (intakeButton.isPressed() || driverIntakeButton.isPressed())
		{
			if (intakeSolenoid.get() == DoubleSolenoid.Value.kReverse)
			{
				intakeSolenoid.set(DoubleSolenoid.Value.kForward);
			}
			else
			{
				intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
			}
		}
		
		if (intakeRollerButton.isPressed() || (intakeSolenoid.get() == DoubleSolenoid.Value.kForward))
		{
			intakeTalon.set(1.0);
		}
		else
		{
			intakeTalon.set(0.0);
		}
		
		if (gearWallUpButton.isPressed())
		{
			gearWall.set(DoubleSolenoid.Value.kForward);
		}
		
		if (gearWallDnButton.isPressed())
		{
			gearWall.set(DoubleSolenoid.Value.kReverse);
		}

		if (gearDrapesButton.isPressed())
		{	
			gearDrapes.set(DoubleSolenoid.Value.kForward);
			gearWall.set(DoubleSolenoid.Value.kReverse);

			if (gearPushoutButton.isPressed())
			{	
				gearPushout.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				gearPushout.set(DoubleSolenoid.Value.kForward);
			}
		}
		else
		{
			gearDrapes.set(DoubleSolenoid.Value.kReverse);
			gearPushout.set(DoubleSolenoid.Value.kForward);
		}

		
		if (feederButton.isPressed())
		{
			feeder.set(-1.0);
		}
		else
		{
			feeder.set(0.0);
		}

		if (shooterButton.isPressed())
		{
			shooter1.set(-0.75);
			shooter2.set(+0.75);
		}
		else
		{
			shooter1.set(0.0);
			shooter2.set(0.0);
		}

		if (shooterHoodButton.isPressed())
		{	
			if (shooterHood.get() == DoubleSolenoid.Value.kForward)
			{
				shooterHood.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				shooterHood.set(DoubleSolenoid.Value.kForward);
			}
		}
		
		//if(i2cButton.isPressed()) {
//			byte a[] = {(byte)0};
			//byte data[] = {'C', (byte)150, 8, 0, (byte)255, 0, 2, 15, 2, 0, 0};
			//byte data[] = {0, 0, 0, 0};
			//SmartDashboard.putBoolean("io expand", i2cIOExpand.writeBulk(data));
			//ByteBuffer dataToSendBuffer = ByteBuffer.allocateDirect(data.length);
		    //dataToSendBuffer.put(data);
			
			//int sent = I2CJNI.i2CWrite((byte) I2C.Port.kOnboard.value, (byte) 0x20, dataToSendBuffer, (byte) data.length);
			//SmartDashboard.putNumber("i2c sent, 14", sent);
			//< data.length;
		//}

		
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
		
		if (unjammerButton.isPressed())
		{
			unjammer.set(1.0);
		}
		else
		{
			unjammer.set(0.0);
		}
		
		//camera();
	}
	
	/**
	 * 
	 */
	private void camera() {
//		if (leftStick.getRawButton(2) && (!cameraButtonHeld))
//		{
//			cameraButtonHeld = true;
//			if (camera1on)
//			{
//				server.setSource(camera2);
//				camera1on = false;
//			}
//			else
//			{
//				server.setSource(camera);
//				camera1on = true;
//			}
//		}
//		else if (!rightStick.getRawButton(2) && (cameraButtonHeld))
//		{
//			cameraButtonHeld = false;
//		}
	}
	
	public double getTime()
	{
		return this.lastTime;
	}
	
	public double getCurrentBearing()
	{
    	double bearing = gyro.getAngle();
    	
    	if (bearing >= 360.0)
    	{
    		bearing = bearing - (360.0 * Math.floor(bearing / 360.0));
    	}
    	else if (bearing <= -360.0)
    	{
    		bearing = bearing + (360.0 * Math.floor(bearing / 360.0));
    	}
    	
    	if (bearing < -180.0)
    	{
    		bearing += 360.0; 
    	}
    	else if (bearing > 180.0)
    	{
    		bearing -= 360.0;
    	}
    	
    	return bearing;
	}
	
	public double getBearing()
	{
		return this.lastBearing;
	}
	
	public double getDistance()
	{
		return this.lastDistance;
	}
	
	public double getVelocity()
	{
		return this.lastVelocity;
	}
	
	public double getAcceleration()
	{
		return this.lastAcceleration;
	}
	
	public void writeDashboard()
	{
		SmartDashboard.putNumber("Bearing", this.getBearing());
		SmartDashboard.putNumber("Distance", this.getDistance());
		SmartDashboard.putNumber("Velocity", this.getVelocity());
		SmartDashboard.putNumber("Acceleration", this.getAcceleration());
		
		SmartDashboard.putNumber("photoelectricSensor", sensor.getVoltage());
		SmartDashboard.putNumber("leftDriveEncoder", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("rightDriveEncoder", rightDriveEncoder.getDistance());

		//cmdSmartDashboard.putNumber("shooterEncoder", shooter2.getEncVelocity());
		//SmartDashboard.putNumber("climberEncoder", climber2.getEncVelocity());

		SmartDashboard.putString("Transmission", shifter.get() == DoubleSolenoid.Value.kForward ? "High" : "Low");
		SmartDashboard.putBoolean("Pickup Out", intakeSolenoid.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Wall Up", gearWall.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Gear Draps Up", gearDrapes.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Gear Push Out", gearPushout.get() == DoubleSolenoid.Value.kForward);
		wheelDrive = SmartDashboard.getBoolean("wheelDrive", true);
		
		double exposure = SmartDashboard.getNumber("Camera Exposure", -3.0);
		if (exposure != currentExposure)
		{
			SmartDashboard.putNumber("Camera Exposure", exposure);
			if (exposure < -2.0)
			{
				if (cameraHigh != null) cameraHigh.setExposureAuto();
				if (cameraLow != null) cameraLow.setExposureAuto();
			}
			else
			{
				if (cameraHigh != null) cameraHigh.setExposureManual((int)exposure);
				if (cameraLow != null) cameraLow.setExposureManual((int)exposure);
			}
			currentExposure = exposure;
		}
		
		double whiteBalance = SmartDashboard.getNumber("Camera WB", -3.0);
		if (whiteBalance != currentWhiteBalance)
		{
			SmartDashboard.putNumber("Camera WB", whiteBalance);
			if (whiteBalance < -2.0)
			{
				if (cameraHigh != null) cameraHigh.setWhiteBalanceAuto();
				if (cameraLow != null) cameraLow.setWhiteBalanceAuto();
			}
			else
			{
				if (cameraHigh != null) cameraHigh.setWhiteBalanceManual((int)whiteBalance);
				if (cameraLow != null) cameraLow.setWhiteBalanceManual((int)whiteBalance);
			}
			currentWhiteBalance = whiteBalance;
		}
		
	}
	
}
