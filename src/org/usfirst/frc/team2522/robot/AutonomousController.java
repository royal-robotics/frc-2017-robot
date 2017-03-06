package org.usfirst.frc.team2522.robot;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MaximizeAction;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/***
 * 
 *
 */
public final class AutonomousController 
{
	public static double driveToLastTime = 0.0;
	public static double driveToStartDistance = 0.0;
	public static double driveStraightBearing = 0.0;
	
	public static boolean driveTo(Robot robot, double bearing, double distance)
	{
		boolean finished = false;

		double leftPower = robot.leftDrive1.get(); 
		double rightPower = robot.rightDrive1.get(); 

		if (driveToLastTime == 0.0) 
		{
			driveToLastTime = robot.getTime();
			leftPower = 0.0;
			rightPower = 0.0;
		}
		else 
		{
			double t = robot.getTime() - driveToLastTime;
					
			if (t >= robot.robotSampleRate)
			{
				MotionPosition p = MotionControl.GetExpectedPosition(t, distance, bearing, 0.0, 150.0, 100.0);
				
				robot.expBearing = p.bearing;
				robot.expDistance = p.distance + driveToStartDistance;
				robot.expVelocity = p.velocity;
				robot.expAcceleration = p.acceleration;
				
				// Set the power to the expected velocity * the velocity feed value.
				leftPower = robot.kVf * p.velocity;
				rightPower = robot.kVf * p.velocity;
				
				// Calculate the current velocity error and apply the proportional error adjustment 
				double vError = robot.getVelocity() - p.velocity;
				leftPower += robot.kVp * vError;
				rightPower += robot.kVp * vError;
				
				// Calculate the current velocity error and apply the proportional error adjustment 
				double aError = robot.getAcceleration() - p.acceleration;
				leftPower += robot.kAp * aError;
				rightPower += robot.kAp * aError;

				// Calculate the current distance error and apply the proportional error adjustment 
				double dError = (robot.getDistance() - driveToStartDistance) - p.distance;
				leftPower += robot.kDp * dError;
				rightPower += robot.kDp * dError;
				
				// Calculate the current bearing error and apply the proportional error adjustment 				
				double bError = robot.getBearing() - p.bearing;
				if (bError > 180.0) 
				{
					bError -= 360.0;
				}
				else if (bError < -180.0)
				{
					bError += 360.0;
				}
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
		
		if ((Math.abs((robot.getDistance() - driveToStartDistance) - distance) < 1.0) && (Math.abs(robot.getVelocity()) < 1.0))
		{
			finished = true;
			leftPower = 0;				
			rightPower = 0;
		}
		
		robot.myDrive.tankDrive(-leftPower, -rightPower);
		
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
