package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/***
 * 
 *
 */
public final class AutonomousController 
{
	private static double maxDec = 3.0;
	
	private static double driveToLastTime = 0.0;
	private static double driveToThrottle = 0.0;
	private static boolean driveToBreaking = false;
	
	public static boolean driveTo(Robot robot, double bearing, double distance)
	{
		boolean finished = false;
		
		if (driveToLastTime == 0.0) 
		{
			driveToLastTime = robot.getTime();
		}
		else 
		{
			double t = robot.getTime() - driveToLastTime;
			double rd = distance - robot.getDistance();
			double eta = rd * robot.getVelocity();
			
			SmartDashboard.putNumber("T", t);
			SmartDashboard.putNumber("RD", rd);
			SmartDashboard.putNumber("ETA", eta);
			
			if (!driveToBreaking)
			{
				if ((eta == 0.0) || ((robot.getVelocity() / eta) <= maxDec))
				{
					if (driveToThrottle < 1.0)
					{
						driveToThrottle = Math.min(driveToThrottle + (t / 0.25), 1.0);
					}
				}
				else
				{
					driveToBreaking = true;
				}
			}
			else
			{
				driveToThrottle = 0.0;
				if (robot.getDistance() >= distance)
				{
					finished = true;
					driveToLastTime = 0.0;
					driveToThrottle = 0.0;
					driveToBreaking = false;
				}
			}
						
			driveForward(robot, bearing, driveToThrottle);
		}
		
		return finished;
	}
	
	public static void driveForward(Robot robot, double bearing, double power)
	{
		double error = bearing - robot.getBearing();
		if (error > 180.0) 
		{
			error -= 360.0;
		}
		else if (error < -180.0)
		{
			error += 360.0;
		}
		
		robot.myDrive.tankDrive(-power - (0.015 * error), -power + (0.015 * error));
	}
}
