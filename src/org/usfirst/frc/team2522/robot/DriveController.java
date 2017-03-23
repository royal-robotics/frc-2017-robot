package org.usfirst.frc.team2522.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveController extends Thread
{
	
	public Robot robot = null;
	public int sleepTime = 5;
	
	public double motionStartTime = 0.0;
	public double motionStartBearing = 0.0;
	public double motionStartLeftDistance = 0.0;
	public double motionStartRightDistance = 0.0;
	public double motionStartVelocity = 0.0;
	public double motionMaxVel = 0.0;
	public double motionMaxAcc = 0.0;
	
	public double motionLastBearingError = 0.0;
	public double motionLastLeftError = 0.0;
	public double motionLastRightError = 0.0;
	
	public double motionLastTime = 0.0;
	
	public double motionDriveDistance = 0.0;
	public double motionRotateAngle = 0.0;
	
	/**
	 * 
	 * @param frequency	Frequency of control loop in Hz 
	 */
	public DriveController(Robot robot, int frequency)
	{
		this.robot = robot;
		this.sleepTime = 1000 / frequency;
	}
	
	public void stopMotion()
	{
		motionStartTime = 0.0;
	}
	
	private void startMotion(double distance, double maxVel, double maxAcc)
	{
		motionStartTime = robot.getTime();	// move this here to prevent drive thread from prematurely ending motion.
		motionStartBearing = robot.getBearing();
		motionStartLeftDistance = robot.leftDriveEncoder.getDistance();
		motionStartRightDistance = robot.rightDriveEncoder.getDistance();
		motionStartVelocity = robot.getVelocity();
		motionLastTime = 0.0;
		
		motionLastBearingError = 0.0;
		motionLastLeftError = 0.0;
		motionLastRightError = 0.0;

		motionMaxVel = maxVel;
		motionMaxAcc = maxAcc;

		if (distance < 0.0)
		{
			motionMaxVel = -motionMaxVel;
			motionMaxAcc = -motionMaxAcc;
		}

		motionDriveDistance = 0.0;
		motionRotateAngle = 0.0;
	}
	
	public void drive(double distance)
	{
		this.drive(distance, 150.0, 100.0);
	}
	
	public void drive(double distance, double maxVel, double maxAcc)
	{
		startMotion(distance, maxVel, maxAcc);
		
		motionDriveDistance = distance;
	}
	
	public void rotate(double angle)
	{
		this.rotate(angle, 350.0, 350.0);
	}
	
	public void rotate(double angle, double maxVel, double maxAcc)
	{
		startMotion(angle, maxVel, maxAcc);
		
		motionRotateAngle = angle;
	}
	
	
	@Override
	public void run()
	{
		while (!Thread.interrupted()) {
			long start = System.currentTimeMillis();
			

			// Update motion / location data.
			//
			double currentTime = robot.robotTimer.get();
			double currentDistance = robot.getCurrentDistance();
			double currentBearing = robot.getCurrentBearing();

			double t = currentTime - robot.lastTime;
			
			// bearing delta
			double b = currentBearing - robot.lastBearing;
//			if (Math.abs(b) < 0.05)
//			{
//				b = 0.0;
//			}
			
			double rv = b / t;
			double ra = (rv - robot.lastRotationalVelocity) / t;
	
			// distance delta
			double d = currentDistance - robot.lastDistance;
//			if (Math.abs(d) < 0.1)
//			{
//				d = 0.0;
//			}
			
			double v = d / t;
			double a = (v - robot.lastVelocity) / t;
	
			
			robot.lastDistance += d;
			robot.lastVelocity = v;		
			robot.lastAcceleration = a;		

			robot.lastBearing += b;
			robot.lastRotationalVelocity = rv;
			robot.lastRotationalAcceleration = ra;

			robot.lastTime = currentTime;
	
			if (robot.recordMotion)
			{
				if (robot.ps == null)
				{
					File f = new File("/home/lvuser/MotionProfile0.txt");
					for(int i = 0;f.exists();i++)
					{
						f = new File("/home/lvuser/MotionProfile" + i + ".txt");
					}
					
					try 
					{
						robot.ps = new PrintStream(f);
					}
					catch(IOException e)
					{
						robot.ps = null;
						e.printStackTrace();
					}
					
					System.out.println("MotionRecording started: " + f.getName());
					robot.motionProfileTimer.reset();
					robot.motionProfileTimer.start();
					
					robot.motionTime = 0.0;
					robot.expBearing = 0.0;
					robot.expLeftDistance = 0.0;
					robot.expRightDistance = 0.0;
					robot.expVelocity = 0.0;
					robot.expAcceleration = 0.0;

					robot.mBearingPError = 0.0;
					robot.mLeftPError = 0.0;
					robot.mRightPError = 0.0;

					robot.mBearingDError = 0.0;
					robot.mLeftDError = 0.0;
					robot.mRightDError = 0.0;

					
					robot.ps.println("Time\t" + 
							   "Motion Time\t" +
							   "Left Power\t" +
							   "Right Power\t" +
							   "Left Distance\t" +
							   "Right Distance\t" +
							   "Exp Left Distance\t" +
							   "Exp Right Distance\t" +
							   "Left ErrorP\t" +
							   "Right ErrorP\t" +
							   "Left ErrorD\t" +
							   "Right ErrorD\t" +
							   "Bearing\t" +
							   "Exp Bearing\t" +
							   "Bearing ErrorP\t" +
							   "Bearing ErrorD\t" +
							   "RV\t" +
							   "RA\t" +
							   "Velocity\t" +
							   "Exp Velocity\t" +
							   "Acceleration\t" + 
							   "Exp Acceleration\t" 
					);
				}
				
				robot.ps.println(String.valueOf(robot.motionProfileTimer.get()) + "\t" + 
						   String.valueOf(robot.motionTime) + "\t" +
						   String.valueOf(robot.leftDrive1.get()) + "\t" +
						   String.valueOf(robot.rightDrive1.get()) + "\t" +
						   String.valueOf(robot.leftDriveEncoder.getDistance()) + "\t" +
						   String.valueOf(robot.rightDriveEncoder.getDistance()) + "\t" +
						   String.valueOf(robot.expLeftDistance) + "\t" +
						   String.valueOf(robot.expRightDistance) + "\t" +
						   String.valueOf(robot.mLeftPError) + "\t" +
						   String.valueOf(robot.mRightPError) + "\t" +
						   String.valueOf(robot.mLeftDError) + "\t" +
						   String.valueOf(robot.mRightDError) + "\t" +
						   String.valueOf(robot.lastBearing) + "\t" +
						   String.valueOf(robot.expBearing) + "\t" +
						   String.valueOf(robot.mBearingPError) + "\t" +
						   String.valueOf(robot.mBearingDError) + "\t" +
						   String.valueOf(robot.getRotationVelocity()) + "\t" +
						   String.valueOf(robot.getRotationAcceleration()) + "\t" +
						   String.valueOf(robot.getVelocity()) + "\t" +
						   String.valueOf(robot.expVelocity) + "\t" +
						   String.valueOf(robot.getAcceleration()) + "\t" +
						   String.valueOf(robot.expAcceleration)
				);
			}
			else
			{
				if (robot.ps != null)
				{
					robot.ps.close();
					robot.ps = null;
					robot.motionProfileTimer.stop();
					robot.motionProfileTimer.reset();
				}
			}
			
			
			
			if (motionStartTime != 0.0) 
			{
				t = robot.getTime() - motionStartTime;

				double leftPower = -robot.leftDrive1.get(); 
				double rightPower = robot.rightDrive1.get(); 
				
				MotionPosition p = null;
				
				if (motionDriveDistance != 0.0)
				{
					p = MotionControl.getExpectedDrivePosition(t, motionDriveDistance, motionStartBearing, 0.0, motionMaxVel, motionMaxAcc);

					// Set the power to the expected velocity * the velocity feed value.
					double power = robot.kVf * p.velocity;
									
					// Adjust power for desired acceleration or breaking
					//
					if ((p.acceleration >= 0.0 && p.velocity >= 0.0) || (p.acceleration <= 0.0 && p.velocity <= 0.0))
					{
						power = power + (robot.kAf * p.acceleration);
					}
					else
					{
						power = power + (robot.kBf * p.acceleration);
					}

					leftPower = power; 
					rightPower = power;

					// Calculate the distance proportional and derivative error.
					//
					double leftError = p.distance - (robot.leftDriveEncoder.getDistance() - motionStartLeftDistance);
					leftPower = leftPower + (robot.kDp * leftError);

					double leftDError = ((leftError - motionLastLeftError) / (t - motionLastTime)) - p.velocity;
					leftPower = leftPower + (robot.kDd * leftDError);

					double rightError = p.distance - (robot.rightDriveEncoder.getDistance() - motionStartRightDistance);
					rightPower = rightPower + (robot.kDp * rightError);

					double rightDError = ((rightError - motionLastRightError) / (t - motionLastTime)) - p.velocity;
					rightPower = rightPower + (robot.kDd * rightDError);

					robot.mLeftPError = leftError;
					robot.mRightPError = rightError;

					robot.mLeftDError = leftDError;
					robot.mRightDError = rightDError;
					
					motionLastLeftError = leftError;
					motionLastRightError = rightError;
					
					motionLastTime = t;

//					if ((p.distance == motionDriveDistance) && (Math.abs((robot.leftDriveEncoder.getDistance() - motionStartLeftDistance) - motionDriveDistance) < 2.0) && (Math.abs(robot.getVelocity()) < 1.0))
					if ((p.distance == motionDriveDistance) &&  (Math.abs(robot.getVelocity()) == 0.0))
					{
						motionStartTime = 0.0;
						leftPower = 0;				
						rightPower = 0;
					}
				}
				else if (motionRotateAngle != 0.0)
				{
					p = MotionControl.getExpectedRotationPosition(t, motionRotateAngle, 0.0, motionMaxVel, motionMaxAcc);
					
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

					// Calculate the distance proportional and derivative error.
					//
					double error = p.bearing - (robot.getBearing() - motionStartBearing);
					robot.mBearingPError = error;
					power += (robot.kRBp * error);

					double dError = ((error - motionLastBearingError) / (t - motionLastTime)) - p.velocity;
					robot.mBearingDError = dError;
					power += (robot.kRBd * dError);

					motionLastBearingError = error;
					motionLastTime = t;
					
					leftPower = power; 
					rightPower = -power;
//System.out.println("p.bearing=" + p.bearing + "motionRotateAngle=" + motionRotateAngle);
					
					if ((Math.abs(p.bearing - motionRotateAngle) < .05) && (Math.abs(robot.getRotationVelocity()) <= 0.05))
					{
						motionStartTime = 0.0;
						leftPower = 0;				
						rightPower = 0;
					}
				}
				else
				{
					motionStartTime = 0.0;
					leftPower = 0;				
					rightPower = 0;
				}
				
				robot.motionTime = t;
				if (p != null)
				{
					robot.expBearing = p.bearing + motionStartBearing;
					robot.expLeftDistance = p.distance + motionStartLeftDistance;
					robot.expRightDistance = p.distance + motionStartRightDistance;
					robot.expVelocity = p.velocity;
					robot.expAcceleration = p.acceleration;
				}

				robot.myDrive.tankDrive(-leftPower, -rightPower, false);
			}
			
			
			//
			//
			long duration = System.currentTimeMillis() - start;
			
			if (duration < sleepTime)
			{
				try {
					Thread.sleep(sleepTime - duration);
				}
				catch (InterruptedException e) {
					break;
				}
			}
		}
	}
}
