package org.usfirst.frc.team2522.robot;

import java.awt.HeadlessException;
import java.io.*;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoException;
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
	
	public DriveController driveController = null;
	
	public final double kVf	= 1.0 / 175.0;	// 1 / max velocity
	public final double kAf = 0.0037;
	public final double kBf = 0.0020;
	public final double kDp = 0.0165;
	public final double kDd = 0.0006;
//	public final double kBp = 0.0015;

	
	public final double kRVf	= 1.0 / 488.0;	// 1 / max rotational velocity
	public final double kRAf = 0.00095; //0.00095;
	public final double kRBf = 0.00035; //0.00035
	public final double kRBp = 0.0028;	//0.00200
	public final double kRBd = 0.00010;
	
	public double lastTime = 0.0;
	public double lastBearing = 0.0;
	public double lastRotationalVelocity = 0.0;
	public double lastRotationalAcceleration = 0.0;

	public double lastDistance = 0.0;
	public double lastVelocity = 0.0;
	public double lastAcceleration = 0.0;
	
	public double driveStraightBearing = 0.0;
	
	public double motionTime = 0.0;
	public double expBearing = 0.0;
	public double expLeftDistance = 0.0;
	public double expRightDistance = 0.0;
	public double expVelocity = 0.0;
	public double expAcceleration = 0.0;
	
	public boolean recordMotion = false;

	public double mBearingPError = 0.0;
	public double mBearingDError = 0.0;
	public double mLeftPError = 0.0;
	public double mRightPError = 0.0;
	public double mLeftDError = 0.0;
	public double mRightDError = 0.0;

	private boolean motionDone = false;
		
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
	
	public RobotDrive myDrive = new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
	
	public DoubleSolenoid shifter = new DoubleSolenoid(0, 3, 4);
	public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(0, 2, 5);
	public DoubleSolenoid gearWall = new DoubleSolenoid(0, 1, 6);
	public DoubleSolenoid gearPushout = new DoubleSolenoid(0, 0, 7);	
	public DoubleSolenoid gearDrapes = new DoubleSolenoid(1, 3, 4);
	public DoubleSolenoid shooterHood = new DoubleSolenoid(1, 2, 5);
		
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
	
	Button switchCameras = new Button(leftStick, 2, ButtonType.Hold);
	Button takeImageButton = new Button(leftStick, 3, ButtonType.Toggle);
	Button motionRecordButton = new Button(leftStick, 7, ButtonType.Hold);
	Button showMaskButton  = new Button(leftStick, 8, ButtonType.Toggle);
	Button showTargetsButton = new Button(leftStick, 9, ButtonType.Toggle);
	
	Button testDriveDistanceButton = new Button(leftStick, 11, ButtonType.Hold);
	Button testDriveRotateButton = new Button(leftStick, 10, ButtonType.Hold);
	
	// Debug Buttons
	Button i2cButton = new Button(leftStick, 11, ButtonType.Toggle);

	Button quickTurnButtonLeft = new Button(rightStick, 5, ButtonType.Hold);
	Button quickTurnButtonRight = new Button(rightStick, 6, ButtonType.Hold);
	Button driverIntakeButton = new Button(rightStick, 2, ButtonType.Toggle);
	
	// Operator Controls
	//
	POVButton intakeButton = new POVButton(operatorStick, 0, 0, ButtonType.Toggle);
	POVButton gearPushoutButton = new POVButton(operatorStick, 0, 90, ButtonType.Hold);
	
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
	
	public UsbCamera cameraHigh = null;
	public UsbCamera cameraLow = null;
	boolean showImageBlobs = false;
	boolean showImageTargets = false;

	Thread visionThread;
	double currentExposure = -99.0;		// keep track of exposure changes so we do not hammer the USB camera with calls
	double currentWhiteBalance = -99.0;	// keep track of white balance changes so we do not hammer the USB camera with calls
	
		
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
    	gearPushout.set(DoubleSolenoid.Value.kReverse);
    	gearDrapes.set(DoubleSolenoid.Value.kReverse);
    	shooterHood.set(DoubleSolenoid.Value.kReverse);

    	// Init Motion Control Values
		//
    	this.resetMotion();

    	
    	// Init Vision Processing Loop
    	//
    	cameraHigh = ImageUtils.getCamera(1,  "High");
    	cameraLow = ImageUtils.getCamera(0,  "Low");    	
    	
    	ImageUtils.setCamera(cameraLow);

    	visionThread = new Thread(() -> {
    		while (!Thread.interrupted()) {
    			if ((cameraLow != null) && (ImageUtils.camera == cameraLow))
    			{
    				ImageUtils.processFrame(2.0, 3.0, showImageBlobs, showImageTargets, takeImageButton.isPressed());
    			}
    			else if ((cameraHigh != null) && (ImageUtils.camera == cameraHigh))
    			{
    				ImageUtils.processFrame(2.0, 3.0, showImageBlobs, showImageTargets, takeImageButton.isPressed());
    			}
    		}
    	});
    	
		visionThread.start();
		
		driveController = new DriveController(this, 200);
		driveController.start();
	}

	/**
	 * 
	 */
	public void robotPeriodic()
	{
		if (motionRecordButton.isPressed())
		{
			this.recordMotion = true;
		}
		else
		{
			this.recordMotion = false;
		}
		
		if (switchCameras.isPressed())
		{
			if ((ImageUtils.camera == cameraLow) && (cameraHigh != null))
			{
				ImageUtils.setCamera(cameraHigh);
			}
			else
			{
				ImageUtils.setCamera(cameraLow);
			}
		}
		
		if (showMaskButton.isPressed())
		{
			showImageBlobs = ! showImageBlobs;
		}
		
		if (showTargetsButton.isPressed())
		{
			showImageTargets = ! showImageTargets;
		}
			
		writeDashboard();		
	}

	/**
	 * 
	 */
	public void autonomousInit()
	{
		this.resetMotion();

		AutonomousController.Initialize(this);
	}

	/**
	 * 
	 */
	public void autonomousPeriodic()
	{
		AutonomousController.Periodic(this);
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
		this.resetMotion();
	}
	
	/**
	 * 
	 */
	public void teleopPeriodic()
	{
		if (testDriveDistanceButton.isPressed())
		{
			double dist = SmartDashboard.getNumber("Test Drive Distance", 60.0);
			SmartDashboard.putNumber("Test Drive Distance", dist);

			if (!this.motionDone)
			{
				driveController.drive(dist);
				this.motionDone = true;
			}

//			if (!this.motionDone)
//			{
//				this.motionDone = AutonomousController.driveTo(this, dist);
//			}
//
			double power = SmartDashboard.getNumber("Test Drive Power", 1.0);
			SmartDashboard.putNumber("Test Drive Power", power);
		}
		else if (testDriveRotateButton.isPressed())
		{
			double dist = SmartDashboard.getNumber("Test Drive Distance", 60.0);
			SmartDashboard.putNumber("Test Drive Distance", dist);

			if (!this.motionDone)
			{
				driveController.rotate(dist);
				this.motionDone = true;
			}

//			if (!this.motionDone)
//			{
//				this.motionDone = AutonomousController.rotateTo(this, dist);
//			}
		}
		else
		{
			this.motionDone = false;
			driveController.stopMotion();

			boolean updateDriveStraightBearing = true;

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
				}
				else
				{
					if (wheelValue < -0.08) 
					{
						leftPower = leftPower * (1+wheelValue);
						myDrive.tankDrive(leftPower, rightPower);
					}
					else if (wheelValue > 0.08)
					{
						rightPower = rightPower * (1 - wheelValue);
						myDrive.tankDrive(leftPower, rightPower);
					}
					else
					{
						if (Math.abs(leftPower) > 0.08)
						{
							driveStraight(this.driveStraightBearing, -leftPower);
							updateDriveStraightBearing = false;
						}
						else
						{
							myDrive.tankDrive(0.0, 0.0);
						}
					}
				}
			}
			
			if (updateDriveStraightBearing)
			{
				this.driveStraightBearing = this.getBearing();
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
				gearPushout.set(DoubleSolenoid.Value.kForward);
			}
			else
			{
				gearPushout.set(DoubleSolenoid.Value.kReverse);
			}
		}
		else
		{
			gearDrapes.set(DoubleSolenoid.Value.kReverse);
			gearPushout.set(DoubleSolenoid.Value.kReverse);
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
			climber1.set(+0.5);
			climber2.set(+0.5);
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
	}
	
	
	public void driveStraight(double bearing, double power)
	{
		double error = bearing - this.getBearing();
		
		this.myDrive.tankDrive(-power - (0.015 * error), -power + (0.015 * error), true);
	}

	public double getTime()
	{
		return this.lastTime;
	}

	public double getCurrentAngle()
	{
    	double angle = gyro.getAngle();
    	
    	if (angle >= 360.0)
    	{
    		angle = angle - (360.0 * Math.floor(angle / 360.0));
    	}
    	else if (angle <= -360.0)
    	{
    		angle = angle + (360.0 * Math.floor(angle / 360.0));
    	}
    	
    	if (angle < -180.0)
    	{
    		angle += 360.0; 
    	}
    	else if (angle > 180.0)
    	{
    		angle -= 360.0;
    	}
    	
    	return angle;
	}
	
	public double getCurrentBearing()
	{
    	return gyro.getAngle();
	}
	
	public double getCurrentDistance()
	{
		return this.leftDriveEncoder.getDistance();
	}

	public double getBearing()
	{
    	return this.lastBearing;
	}
	
	public double getRotationVelocity()
	{
		return this.lastRotationalVelocity;
	}
	
	public double getRotationAcceleration()
	{
		return this.lastRotationalAcceleration;
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
	
	public void resetMotion()
	{
		this.gyro.reset();
		this.lastBearing = this.getCurrentBearing();
		
		this.lastRotationalVelocity = 0.0;
		this.lastRotationalAcceleration = 0.0;
		
		this.leftDriveEncoder.reset();
		this.rightDriveEncoder.reset();
		this.lastDistance = 0.0;
		
		this.lastVelocity = 0.0;
		this.lastAcceleration = 0.0;
		
		this.lastTime = this.robotTimer.get();
		
		
		this.driveStraightBearing = 0.0;
	}
	
	public void writeDashboard()
	{
		wheelDrive = SmartDashboard.getBoolean("wheelDrive", true);

		AutoRoutine autoRoutine = AutonomousController.getAutoRoutine();
		if (autoRoutine != null)
		{
			SmartDashboard.putString("AutoMode", autoRoutine.getName());
			SmartDashboard.putNumber("AutoStep", autoRoutine.autoStep);
		}
		else 
		{
			SmartDashboard.putString("AutoMode", "Do Nothing");
			SmartDashboard.putNumber("AutoStep", 0);
		}
		
		SmartDashboard.putNumber("Angle", this.getCurrentAngle());
		SmartDashboard.putNumber("Bearing", this.getBearing());
		SmartDashboard.putNumber("RotVelocity", this.getRotationVelocity());
		SmartDashboard.putNumber("RotAcceleration", this.getRotationAcceleration());
		
		SmartDashboard.putNumber("Distance", this.getDistance());
		SmartDashboard.putNumber("Velocity", this.getVelocity());
		SmartDashboard.putNumber("Acceleration", this.getAcceleration());
		
		SmartDashboard.putNumber("photoelectricSensor", sensor.getVoltage());
		
		SmartDashboard.putNumber("leftDrivePower", leftDrive1.get());
		SmartDashboard.putNumber("leftDriveEncoder", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("rightDrivePower", rightDrive1.get());
		SmartDashboard.putNumber("rightDriveEncoder", rightDriveEncoder.getDistance());

		SmartDashboard.putNumber("shooterEncoder", shooter2.getEncVelocity());
		//SmartDashboard.putNumber("climberEncoder", climber2.getEncVelocity());

		SmartDashboard.putString("Transmission", shifter.get() == DoubleSolenoid.Value.kForward ? "High" : "Low");
		SmartDashboard.putBoolean("Pickup Out", intakeSolenoid.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Wall Up", gearWall.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Gear Draps Up", gearDrapes.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Gear Push Out", gearPushout.get() == DoubleSolenoid.Value.kReverse);
		
		//
		// Camera Settings
		//
		double exposure = SmartDashboard.getNumber("Camera Exposure", 1.0);
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
		SmartDashboard.putNumber("Camera Exposure", exposure);
		
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
		SmartDashboard.putNumber("Camera WB", whiteBalance);
		
		int hsvLowerHue = (int)SmartDashboard.getNumber("H Lower", 45.0);
		int hsvLowerSat = (int)SmartDashboard.getNumber("S Lower", 50.0);
		int hsvLowerVal = (int)SmartDashboard.getNumber("V Lower", 100.0);		
		if ((ImageUtils.hsvLowerBounds.val[0] != hsvLowerHue) || (ImageUtils.hsvLowerBounds.val[1] != hsvLowerSat) || (ImageUtils.hsvLowerBounds.val[2] != hsvLowerVal))
		{
			ImageUtils.hsvLowerBounds = new Scalar(hsvLowerHue, hsvLowerSat, hsvLowerVal, 0);
		}
		SmartDashboard.putNumber("H Lower", hsvLowerHue);
		SmartDashboard.putNumber("S Lower", hsvLowerSat);
		SmartDashboard.putNumber("V Lower", hsvLowerVal);
		
		int hsvUpperHue = (int)SmartDashboard.getNumber("H Upper", 75.0);
		int hsvUpperSat = (int)SmartDashboard.getNumber("S Upper", 255.0);
		int hsvUpperVal = (int)SmartDashboard.getNumber("V Upper", 255.0);
		if ((ImageUtils.hsvUpperBounds.val[0] != hsvUpperHue) || (ImageUtils.hsvUpperBounds.val[1] != hsvUpperSat) || (ImageUtils.hsvUpperBounds.val[2] != hsvUpperVal))
		{
			ImageUtils.hsvUpperBounds = new Scalar(hsvUpperHue, hsvUpperSat, hsvUpperVal, 0);
		}
		SmartDashboard.putNumber("H Upper", hsvUpperHue);
		SmartDashboard.putNumber("S Upper", hsvUpperSat);
		SmartDashboard.putNumber("V Upper", hsvUpperVal);
	}
	
}
