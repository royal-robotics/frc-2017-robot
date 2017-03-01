package org.usfirst.frc.team2522.robot;

import java.io.*;
import java.nio.ByteBuffer;

import org.opencv.core.Mat;
import org.usfirst.frc.team2522.robot.Button.ButtonType;

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
	Thread visionThread;
	
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
	
	RobotDrive myDrive = new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
	
	DoubleSolenoid shifter = new DoubleSolenoid(0, 3, 4);
	DoubleSolenoid intake = new DoubleSolenoid(0, 2, 5);
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
	
	Button shiftButton = new Button(leftStick, 1, Button.ButtonType.Toggle);
	Button cameraLowButton = new Button(leftStick, 2, Button.ButtonType.Hold);

	// Debug Buttons
	Button i2cButton = new Button(leftStick, 11, Button.ButtonType.Toggle);
	Button motionRecordButton = new Button(leftStick, 7, Button.ButtonType.Hold);
	
	Button quickTurnButtonLeft = new Button(rightStick, 5, Button.ButtonType.Hold);
	Button quickTurnButtonRight = new Button(rightStick, 6, Button.ButtonType.Hold);

	Button intakeButton = new Button(operatorStick, 1, Button.ButtonType.Toggle);
	Button gearWallButton = new Button(operatorStick, 2, Button.ButtonType.Toggle);
	Button gearPushoutButton = new Button(operatorStick, 3, Button.ButtonType.Toggle);
	Button gearDrapesButton = new Button(operatorStick, 4, Button.ButtonType.Toggle);
	Button shooterHoodButton = new Button(operatorStick, 5, Button.ButtonType.Toggle);
	
	Button shooterButton = new Button(operatorStick, 6, Button.ButtonType.Hold);
	Button pickupButton = new Button(operatorStick, 7, Button.ButtonType.Hold);
	Button climberButton = new Button(operatorStick, 9, Button.ButtonType.Hold);
	Button feederButton = new Button(operatorStick, 8, Button.ButtonType.Hold);
	Button unjammerButton = new Button(operatorStick, 10, Button.ButtonType.Hold);
	
	
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
		myDrive.setSafetyEnabled(false);
		leftDrive1.setSafetyEnabled(false);
		leftDrive2.setSafetyEnabled(false);
		rightDrive1.setSafetyEnabled(false);
		rightDrive2.setSafetyEnabled(false);
		
		pickup.setSafetyEnabled(false);
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
		
		leftDriveEncoder.setReverseDirection(true);
		leftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
		leftDriveEncoder.reset();
		rightDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
		rightDriveEncoder.reset();
		
		shooter2.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		shooter2.configEncoderCodesPerRev(1024);//This is wrong? shooterDistancePerPulse?
		climber2.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		climber2.configEncoderCodesPerRev(1024);//This is wrong? shooterDistancePerPulse?
    	
    	shifter.set(DoubleSolenoid.Value.kForward);
    	intake.set(DoubleSolenoid.Value.kReverse);
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
	}

	/**
	 * 
	 */
	public void robotPeriodic()
	{
		writeDashboard();
	}

	/**
	 * 
	 */
	public void autonomousInit()
	{
		
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

	}
	
	/**
	 * 
	 */
	public void teleopPeriodic()
	{
		if (!wheelDrive)
		{
			myDrive.tankDrive(leftStick, rightStick, true);
		}
		else
		{
			double rightValue = leftStick.getY();
			double leftValue = leftStick.getY();			
			double rightX = rightStick.getX();
			
			if (quickTurnButtonLeft.isPressed() || quickTurnButtonRight.isPressed())
			{
				rightValue = rightX;
				leftValue = -rightX;
			}
			else
			{
				if (rightX < -0.05) 
				{
					leftValue = leftValue * (1+rightX);
				}
				else if (rightX > 0.05)
				{
					rightValue = rightValue * (1 - rightX);
				}
			}
			
			myDrive.tankDrive(leftValue, rightValue);
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
					   String.valueOf(leftDriveEncoder.getDistance()) + "," +
					   String.valueOf(gyro.getAngle()) + "," +
					   String.valueOf(gyro.getVelocityX()) + "," +
					   String.valueOf(gyro.getVelocityY()) + "," +
					   String.valueOf(gyro.getVelocityZ())
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
		
		if (intakeButton.isPressed())
		{
			if (intake.get() == DoubleSolenoid.Value.kForward)
			{
				intake.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				intake.set(DoubleSolenoid.Value.kForward);
			}
		}
		
		if (gearWallButton.isPressed())
		{
			if (gearWall.get() == DoubleSolenoid.Value.kForward)
			{
				gearWall.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				gearWall.set(DoubleSolenoid.Value.kForward);
			}
		}

		if (gearPushoutButton.isPressed())
		{	
			if (gearPushout.get() == DoubleSolenoid.Value.kForward)
			{
				gearPushout.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				gearPushout.set(DoubleSolenoid.Value.kForward);
			}
		}

		if (gearDrapesButton.isPressed())
		{	
			if (gearDrapes.get() == DoubleSolenoid.Value.kForward)
			{
				gearDrapes.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				gearDrapes.set(DoubleSolenoid.Value.kForward);
			}
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
			feeder.set(-1.0);
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
	
	public void writeDashboard()
	{
		SmartDashboard.putNumber("photoelectricSensor", sensor.getVoltage());
		SmartDashboard.putNumber("gyro", gyro.getAngle());
		SmartDashboard.putNumber("leftDriveEncoder", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("rightDriveEncoder", rightDriveEncoder.getDistance());
		SmartDashboard.putString("Transmission", shifter.get() == DoubleSolenoid.Value.kForward ? "High" : "Low");

		//cmdSmartDashboard.putNumber("shooterEncoder", shooter2.getEncVelocity());
		//SmartDashboard.putNumber("climberEncoder", climber2.getEncVelocity());
		SmartDashboard.putBoolean("Pickup Out", intake.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Wall Up", gearWall.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Gear Draps Up", gearDrapes.get() == DoubleSolenoid.Value.kForward);
		SmartDashboard.putBoolean("Gear Push Out", gearPushout.get() == DoubleSolenoid.Value.kForward);
		wheelDrive = SmartDashboard.getBoolean("wheelDrive", true);
		
		double exposure = SmartDashboard.getNumber("Camera Exposure", -1.0);
		SmartDashboard.putNumber("Camera Exposure", exposure);
		double whiteBalance = SmartDashboard.getNumber("Camera WB", -1.0);
		SmartDashboard.putNumber("Camera WB", whiteBalance);
		if (exposure < 0.0)
		{
			if (cameraHigh != null) cameraHigh.setExposureAuto();
			if (cameraLow != null) cameraLow.setExposureAuto();
		}
		else
		{
			if (cameraHigh != null) cameraHigh.setExposureManual((int)exposure);
			if (cameraLow != null) cameraLow.setExposureManual((int)exposure);
		}
		
		if (whiteBalance < 0.0)
		{
			if (cameraHigh != null) cameraHigh.setWhiteBalanceAuto();
			if (cameraLow != null) cameraLow.setWhiteBalanceAuto();
		}
		else
		{
			if (cameraHigh != null) cameraHigh.setWhiteBalanceManual((int)whiteBalance);
			if (cameraLow != null) cameraLow.setWhiteBalanceManual((int)whiteBalance);
		}
		
	}
	
}
