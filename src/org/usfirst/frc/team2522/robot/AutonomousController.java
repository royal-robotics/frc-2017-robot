package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/***
 * 
 *
 */
public final class AutonomousController 
{
	public static double driveToLastTime = 0.0;
	public static double driveToThrottle = 0.0;
	
	public static boolean driveTo(Robot robot, double bearing, double distance)
	{
		boolean finished = false;

		if (driveToLastTime == 0.0) 
		{
			driveToLastTime = robot.getTime();
			driveToThrottle = 0.0;
		}
		else 
		{
			double t = robot.getTime() - driveToLastTime;
			double rd = distance - robot.getDistance();
			double eta = robot.getVelocity() == 0.0 ? 60.0 : rd / robot.getVelocity();
			if (eta < 0.0)
			{
				eta = 60.0;
			}

			double ets = (-0.00004052 * robot.getVelocity() * robot.getVelocity()) + (0.011536 * Math.abs(robot.getVelocity())) + 0.480686;

			
			
			if ((Math.abs(rd) < 1.0) && (Math.abs(robot.getVelocity()) == 0.0))
//			if ((robot.getDistance() > distance) && (robot.getVelocity() == 0.0))
			{
				driveToThrottle = 0.0;
				driveToLastTime = 0.0;
				finished = true;
			}
			else
			{
//				if (robot.getDistance() > distance)
//				{
//					// throttle down
//					if (driveToThrottle > 0.0)
//					{
//						driveToThrottle = Math.max(driveToThrottle - (t / 0.25), 0.0);
//					}
//				}
//				else
//				{
//					// throttle up
//					if (driveToThrottle < 1.0)
//					{
//						driveToThrottle = Math.min(driveToThrottle + (t / 0.25), 1.0);
//					}
//				}
				
				if (eta >= ets)
				{
					// throttle up
					if (rd > 0.0)
					{
						if (driveToThrottle < 1.0)
						{
							driveToThrottle = Math.min(driveToThrottle + (t / 0.25), 1.0);
						}
					}
					else
					{
						if (driveToThrottle > -1.0)
						{
							driveToThrottle = Math.max(driveToThrottle - (t / 0.25), -1.0);
						}
					}
				}
				else
				{
					// throttle down
					if (driveToThrottle > 0.0)
					{
						driveToThrottle = Math.max(driveToThrottle - (t / 0.25), 0.0);
					}
					else if (driveToThrottle < 0.0)
					{
						driveToThrottle = Math.min(driveToThrottle + (t / 0.25), 0.0);
					}
				}
			}
			
			driveForward(robot, bearing, driveToThrottle);
			driveToLastTime += t;
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
