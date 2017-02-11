package org.usfirst.frc.team2522.robottest;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;
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
public class driveSystem extends IterativeRobot
{
	VictorSP frontLeftDrive = new VictorSP(0);
	VictorSP frontRightDrive = new VictorSP(1);
	VictorSP backLeftDrive = new VictorSP(2);
	VictorSP backRightDrive = new VictorSP(3);
	
	CANTalon leftShooter = new CANTalon(0);
	CANTalon rightShooter = new CANTalon(1);
	
	CANTalon pickup = new CANTalon(2);
	
	CANTalon leftClimber = new CANTalon(3);
	CANTalon rightClimber = new CANTalon(4);
	
	CANTalon feeder = new CANTalon(5);

	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	
	Encoder frontLeftDriveEncoder = new Encoder(new DigitalInput(2), new DigitalInput(3));
	Encoder frontRightDriveEncoder = new Encoder(new DigitalInput(4), new DigitalInput(5));
	
	Encoder shooterEncoder = new Encoder(new DigitalInput(6), new DigitalInput(7));
	
	RobotDrive myDrive;
	
	DoubleSolenoid leftShifter = new DoubleSolenoid(0, 0, 1);
	DoubleSolenoid rightShifter = new DoubleSolenoid(0, 2, 3);
	
	DoubleSolenoid intake = new DoubleSolenoid(0, 4, 5);
	DoubleSolenoid ballPit = new DoubleSolenoid(0, 6, 7);
	DoubleSolenoid gearPushout = new DoubleSolenoid(1, 0, 1);
	DoubleSolenoid drapes = new DoubleSolenoid(1, 2, 3);
	DoubleSolenoid shooterHood = new DoubleSolenoid(1, 4, 5);
	
	DigitalOutput LED = new DigitalOutput(0);
	
	AnalogInput sensor = new AnalogInput(0);
	
	//  (<Wheel Diameter in Inches> * <Pi>) / (<Pulses Per Rotation> * <Encoder Mount Gearing> <Third Stage Gearing>)  //
//	public static double driveTranDistancePerPulse = ( * 3.1415) / ( * ( / ) * ( / ));
	
	public void robotInit()
	{
//      frontLeftDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
//      frontLeftDriveEncoder.reset();
//		frontRightDriveEncoder.setDistancePerPulse(driveTranDistancePerPulse);
//		frontRightDriveEncoder.reset();
		
    	myDrive = new RobotDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
		
//    	shooterEncoder.setDistancePerPulse(shooterDistancePerPulse);
//    	shooterEncoder.reset();
    	
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
	boolean shiftedButtonHeld = false;
	boolean intakeOut = false;
	boolean intakeButtonHeld = false;
	boolean ballPitUp = false;
	boolean ballPitButtonHeld = false;
	boolean gearPushin = false;
	boolean gearPushoutButtonHeld = false;
	boolean draped = false;
	boolean drapesButtonHeld = false;
	boolean shooterHoodUp = false;
	boolean shooterButtonHeld = false;
	boolean LEDOn = false;
	boolean LEDButtonHeld = false;
	
	public void teleopPeriodic()
	{
		myDrive.tankDrive(leftStick, rightStick, true);
		
		if (leftStick.getRawButton(1) && (!shiftedButtonHeld))
		{	
			if (shifted)
			{
				leftShifter.set(DoubleSolenoid.Value.kReverse);
				rightShifter.set(DoubleSolenoid.Value.kReverse);
				shifted = false;
				shiftedButtonHeld = true;
			}
			else
			{
				leftShifter.set(DoubleSolenoid.Value.kForward);
				rightShifter.set(DoubleSolenoid.Value.kForward);
				shifted = true;
				shiftedButtonHeld = true;
			}
		}
		else
		{
			shiftedButtonHeld = false;
		}
		
		if (rightStick.getRawButton(1) && (!intakeButtonHeld))
		{	
			if (intakeOut)
			{
				intake.set(DoubleSolenoid.Value.kReverse);
				intakeOut = false;
				intakeButtonHeld = true;
			}
			else
			{
				intake.set(DoubleSolenoid.Value.kForward);
				intakeOut = true;
				intakeButtonHeld = true;
			}
		}
		else
		{
			intakeButtonHeld = false;
		}
		
		if (rightStick.getRawButton(2) && (!ballPitButtonHeld))
		{	
			if (ballPitUp)
			{
				ballPit.set(DoubleSolenoid.Value.kReverse);
				ballPitUp = false;
				ballPitButtonHeld = true;
			}
			else
			{
				ballPit.set(DoubleSolenoid.Value.kForward);
				ballPitUp = true;
				ballPitButtonHeld = true;
			}
		}
		else
		{
			ballPitButtonHeld = false;
		}
		
		if (rightStick.getRawButton(3) && (!gearPushoutButtonHeld))
		{	
			if (intakeOut)
			{
				gearPushout.set(DoubleSolenoid.Value.kReverse);
				gearPushin = false;
				gearPushoutButtonHeld = true;
			}
			else
			{
				gearPushout.set(DoubleSolenoid.Value.kForward);
				gearPushin = true;
				gearPushoutButtonHeld = true;
			}
		}
		else
		{
			gearPushoutButtonHeld = false;
		}
		
		if (rightStick.getRawButton(4) && (!drapesButtonHeld))
		{	
			if (draped)
			{
				drapes.set(DoubleSolenoid.Value.kReverse);
				draped = false;
				drapesButtonHeld = true;
			}
			else
			{
				drapes.set(DoubleSolenoid.Value.kForward);
				draped = true;
				drapesButtonHeld = true;
			}
		}
		else
		{
			drapesButtonHeld = false;
		}
		
		if (rightStick.getRawButton(5) && (!shooterButtonHeld))
		{	
			if (shooterHoodUp)
			{
				shooterHood.set(DoubleSolenoid.Value.kReverse);
				shooterHoodUp = false;
				shooterButtonHeld = true;
			}
			else
			{
				shooterHood.set(DoubleSolenoid.Value.kForward);
				shooterHoodUp = true;
				shooterButtonHeld = true;
			}
		}
		else
		{
			shooterButtonHeld = false;
		}
		
		if (leftStick.getRawButton(2))
		{
			leftShooter.set(1.0);
			rightShooter.set(1.0);
		}
		else
		{
			leftShooter.set(0.0);
			rightShooter.set(0.0);
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
			leftClimber.set(1.0);
			rightClimber.set(1.0);
		}
		else
		{
			leftClimber.set(0.0);
			rightClimber.set(0.0);
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
		
		if (rightStick.getRawButton(6) && (!LEDButtonHeld))
		{
			if (LEDOn)
			{
				LED.set(true);
				LEDOn = true;
				LEDButtonHeld = true;
			}
			else
			{
				LED.set(false);
				LEDOn = true;
				LEDButtonHeld = true;
			}
		}
		else
		{
			LEDButtonHeld = false;
		}
		
	}
	
}
