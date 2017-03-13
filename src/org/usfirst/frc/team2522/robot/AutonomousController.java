package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/***
 * 
 *
 */
public final class AutonomousController 
{
	public static double motionStartTime = 0.0;
	public static double motionStartDistance = 0.0;
	public static double motionStartBearing = 0.0;
	public static double motionLastError = 0.0;
	public static double motionLastTime = 0.0;

	public static int autoMode = 0;
	public static int autoStep = 0;

	public static double autoRotation = 0.0;
	public static boolean autoIsDriving = false;

	public static int getAutoMode()
	{
		return 1;
	}
	

	public static void periodic(Robot robot)
	{
		if (getAutoMode() == 1)
		{
			auto_1_1(robot);
		}
		else if (getAutoMode() == 2)
		{
			auto_2_1(robot);
		}
	}
	
	/**
	 * 
	 * @param robot
	 */
	public static void auto_1_1(Robot robot)
	{
		if (autoStep == 0) 
		{
			if (driveTo(robot, 68.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 1)
		{
			if (rotateTo(robot, -60.0, 200, 450))
			{
				autoStep++;
			}
		}
		else if (autoStep == 2) 
		{
			ImageUtils.setCamera(robot.cameraLow);
			autoRotation = ImageUtils.getPegRotationError();
			while(autoRotation == -999.0)
			{
				autoRotation = ImageUtils.getPegRotationError();
			}
			autoStep++;
		}
		else if (autoStep == 3)
		{
			if (rotateTo(robot, autoRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 4) 
		{
			if (driveTo(robot, 50.0))
			{
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				autoStep++;
			}
		}
		else if (autoStep == 5) 
		{
			if (driveTo(robot, 20.0, 150, 80.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 6) 
		{
			robot.gearDrapes.set(DoubleSolenoid.Value.kForward);
			try {
				Thread.sleep(350);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			autoStep++;
		}
		else if (autoStep == 7) 
		{
			robot.gearPushout.set(DoubleSolenoid.Value.kReverse);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			autoStep++;
		}
		else if (autoStep == 8) 
		{
			if (driveTo(robot, -5.0, 35.0, 50.0))
			{
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				autoStep++;
			}
		}
		else if (autoStep == 9) 
		{
			if (driveTo(robot, -30.0, 35.0, 35.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 10) 
		{
			if (rotateTo(robot, 180, 200, 450))
			{
				autoStep++;
			}
		}
		else if (autoStep == 11) 
		{
			if (driveTo(robot, 80.0))
			{
				autoStep++;
			}
		}
		else
		{
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}
	
	public static void auto_2_1(Robot robot)
	{
		if (autoStep == 0)
		{
			if (driveTo(robot, 24.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 1) 
		{
			ImageUtils.setCamera(robot.cameraLow);
			autoRotation = ImageUtils.getPegRotationError();
			while(autoRotation == -999.0)
			{
				autoRotation = ImageUtils.getPegRotationError();
			}
			autoStep++;
		}
		else if (autoStep == 2)
		{
			if (rotateTo(robot, autoRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 3) 
		{
			if (driveTo(robot, 52.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 4) 
		{
			robot.gearDrapes.set(DoubleSolenoid.Value.kForward);
			try {
				Thread.sleep(250);
				autoStep++;
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		else if (autoStep == 5) 
		{
			robot.gearPushout.set(DoubleSolenoid.Value.kReverse);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			autoStep++;
		}
		else if (autoStep == 6) 
		{
			if (driveTo(robot, -5.0, 35.0, 50.0))
			{
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				autoStep++;
			}
		}
		else if (autoStep == 7) 
		{
			if (driveTo(robot, -30.0, 35.0, 35.0))
			{
				autoStep++;
			}
		}
		else
		{
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}
	
	public static boolean rotateTo(Robot robot, double angle)
	{
		return rotateTo(robot, angle, 200.0, 300.0);
	}
	
	public static boolean rotateTo(Robot robot, double angle, double maxVel, double maxAcc)
	{
		if (!autoIsDriving)
		{
			robot.driveController.rotate(angle, maxVel, maxAcc);
			autoIsDriving = true;
		}
		else
		{
			autoIsDriving = (robot.driveController.motionStartTime != 0.0);
		}
		
		return !autoIsDriving;
	}

	public static boolean driveTo(Robot robot, double distance)
	{
		return driveTo(robot, distance, 150, 100);
	}
	
	public static boolean driveTo(Robot robot, double distance, double maxVel, double maxAcc)
	{
		if (!autoIsDriving)
		{
			robot.driveController.drive(distance, maxVel, maxAcc);
			autoIsDriving = true;
		}
		else
		{
			autoIsDriving = (robot.driveController.motionStartTime != 0.0);
		}
		
		return !autoIsDriving;
	}
	
	
	public static void driveForward(Robot robot, double bearing, double power)
	{
		double error = bearing - robot.getBearing();
		
		robot.myDrive.tankDrive(-power - (0.015 * error), -power + (0.015 * error), true);
	}
	
}
