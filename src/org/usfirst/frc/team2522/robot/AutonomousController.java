package org.usfirst.frc.team2522.robot;

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
	
	public static boolean rotateTo(Robot robot, double angle)
	{
		boolean finished = false;
		
		double leftPower = robot.leftDrive1.get(); 
		double rightPower = robot.rightDrive1.get(); 

		if (motionStartTime == 0.0) 
		{
			motionStartTime = robot.getTime();
			leftPower = 0.0;
			rightPower = 0.0;
		}
		else 
		{
			double t = robot.getTime() - motionStartTime;
			
			if (t >= robot.robotSampleRate)
			{
				double maxRAcc = 300.0;
				
				if (angle < 0.0)
				{
					maxRAcc = -maxRAcc;
				}
				
				MotionPosition p = MotionControl.getExpectedRotationPosition(t, angle, maxRAcc);
				
				robot.motionTime = t;
				robot.expBearing = p.bearing + motionStartBearing;
				robot.expDistance = 0.0;
				robot.expVelocity = p.velocity;
				robot.expAcceleration = p.acceleration;
				
				// Set the power to the expected velocity * the velocity feed value.
				double power = robot.kRVf * p.velocity;
								
				// Adjust power for desired acceleration or breaking
				//
				if ((p.acceleration > 0.0 && p.velocity > 0.0) || (p.acceleration < 0.0 && p.velocity < 0.0))
				{
					power += robot.kRAf * p.acceleration;
				}
				else
				{
					power += robot.kRBf * p.acceleration;
				}

				// Calculate the current velocity and distance errors and average the adjustment between them.
				//
				double vError = robot.getRotationVelocity() - p.velocity;
				double dError = (robot.getBearing() - motionStartBearing) - p.bearing;
				power += ((robot.kRVp * vError) + (robot.kRBp * dError)) / 2.0;
				
				leftPower = power; 
				rightPower = -power;
			}
		}		
		
//		if ((Math.abs((robot.getRawBearing() - motionStartBearing) - angle) < 1.0) && (Math.abs(robot.getRotationVelocity()) < 1.0))
//		{
//			finished = true;
//			leftPower = 0;				
//			rightPower = 0;
//		}
		
		robot.myDrive.tankDrive(-leftPower, -rightPower, false);

		motionStartDistance = robot.getDistance();
		
		
		return finished;
	}

	
	public static boolean driveTo(Robot robot, double distance)
	{
		boolean finished = false;

		double leftPower = robot.leftDrive1.get(); 
		double rightPower = robot.rightDrive1.get(); 

		if (motionStartTime == 0.0) 
		{
			motionStartTime = robot.getTime();
			leftPower = 0.0;
			rightPower = 0.0;
		}
		else 
		{
			double t = robot.getTime() - motionStartTime;
					
			if (t >= robot.robotSampleRate)
			{
				double maxVel = 150.0;
				double maxAcc = 100.0;
				
				if (distance < 0.0)
				{
					maxVel = -maxVel;
					maxAcc = -maxAcc;
				}
				
				MotionPosition p = MotionControl.getExpectedDrivePosition(t, distance, motionStartBearing, 0.0, maxVel, maxAcc);
				
				robot.motionTime = t;
				robot.expBearing = p.bearing;
				robot.expDistance = p.distance + motionStartDistance;
				robot.expVelocity = p.velocity;
				robot.expAcceleration = p.acceleration;
				
				// Set the power to the expected velocity * the velocity feed value.
				double power = robot.kVf * p.velocity;
								
				// Adjust power for desired acceleration or breaking
				//
				if ((p.acceleration > 0.0 && p.velocity > 0.0) || (p.acceleration < 0.0 && p.velocity < 0.0))
				{
					power += robot.kAf * p.acceleration;
				}
				else
				{
					power += robot.kBf * p.acceleration;
				}

				// Calculate the current velocity and distance errors and average the adjustment between them.
				//
				double vError = robot.getVelocity() - p.velocity;
				double dError = (robot.getDistance() - motionStartDistance) - p.distance;
				power += ((robot.kVp * vError) + (robot.kDp * dError)) / 2.0;
				
				leftPower = power; 
				rightPower = power;
				
				// Calculate the current bearing error and apply the proportional error adjustment 				
				double bError = robot.getBearing() - p.bearing;
				double leftAdj = -robot.kBp * bError;
				double rightAdj = robot.kBp * bError;
				
				if (leftPower + leftAdj > 1.0)
				{
					rightAdj -= leftAdj - (1.0 - leftPower);
					leftAdj -= leftAdj - (1.0 - leftPower);
				}
				
				if (leftPower + leftAdj < -1.0)
				{
					rightAdj -= leftAdj - (-1.0 - leftPower);
					leftAdj -= leftAdj - (-1.0 - leftPower);
				}
				
				if (rightPower + rightAdj > 1.0)
				{
					rightAdj -= rightAdj - (1.0 - rightPower);
					leftAdj -= rightAdj - (1.0 - rightPower);
				}
				
				if (rightPower + rightAdj < -1.0)
				{
					rightAdj -= rightAdj - (-1.0 - rightPower);
					leftAdj -= rightAdj - (-1.0 - rightPower);
				}
					
				leftPower += leftAdj;				
				rightPower += rightAdj;
			}
		}
		
//		if ((Math.abs((robot.getDistance() - motionStartDistance) - distance) < 1.0) && (Math.abs(robot.getVelocity()) < 1.0))
//		{
//			finished = true;
//			leftPower = 0;				
//			rightPower = 0;
//		}
		
		robot.myDrive.tankDrive(-leftPower, -rightPower, false);
		
		return finished;
	}
	
	
	public static void driveForward(Robot robot, double bearing, double power)
	{
		double error = bearing - robot.getBearing();
		
		robot.myDrive.tankDrive(-power - (0.015 * error), -power + (0.015 * error), true);
	}
	
}
