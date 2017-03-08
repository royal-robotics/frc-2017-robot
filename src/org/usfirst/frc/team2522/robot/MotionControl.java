package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotionControl 
{
	/**
	 * 
	 * @param t
	 * @param distance
	 * @param bearing
	 * @param vi
	 * @param maxVel
	 * @param maxAccl
	 * 
	 * @return
	 */
	public static MotionPosition getExpectedDrivePosition(double t, double distance, double bearing, double vi, double maxVel, double maxAccl)
	{
		/**
		 * Maximum deceleration value.
		 */
		double maxBreak = -maxAccl;
				
				
		/**
		 * Velocity at beginning of acceleration.
		 */
		double accl_vi = vi;
		
		/**
		 * Velocity at end of acceleration.
		 */
		double accl_vf = maxVel;
		
		/**
		 * Time spent accelerating.
		 */
		double accl_t = getTimeToVelocity(accl_vf, accl_vi, maxAccl);
		
		/**
		 * Distance traveled while accelerating.
		 */
		double accl_d = getDistance(accl_t, accl_vi, maxAccl);
		
		/**
		 * Velocity at beginning of breaking.
		 */
		double break_vi = accl_vf;
		
		/**
		 * Velocity at end of breaking.
		 */
		double break_vf = 0.0;
		
		/**
		 * Time spent breaking.
		 */
		double break_t = getTimeToVelocity(break_vf, break_vi, maxBreak);
		
		/**
		 * Distance traveled while breaking.
		 */
		double break_d = getDistance(break_t, break_vi, maxBreak);
				
		/**
		 * Time spent at cruise speed.
		 */
		double cruise_t = 0.0;
		
		/**
		 * Distance traveled at cruise speed.
		 */
		double cruise_d = 0.0;
		
		
		if (Math.abs(distance) >= Math.abs(accl_d + break_d))
		{
			cruise_d = distance - (accl_d + break_d);
			cruise_t = getTimeToDistance(cruise_d, accl_vf, 0.0);
		}
		else
		{
			accl_d =  (maxAccl * distance) / (maxAccl - maxBreak);
			accl_t = getTimeToDistance(accl_d, accl_vi, maxAccl);
			accl_vf = getVelocity(accl_t, accl_vi, maxAccl);
			
			break_vi = accl_vf;
			break_d = distance - accl_d;
			break_t = getTimeToVelocity(break_vf, break_vi, maxBreak);
		}
	SmartDashboard.putNumber("accl_t", accl_t);
	SmartDashboard.putNumber("accl_d", accl_d);
	SmartDashboard.putNumber("cruise_t", cruise_t);
	SmartDashboard.putNumber("cruise_d", cruise_d);
	SmartDashboard.putNumber("break_t", break_t);
	SmartDashboard.putNumber("break_d", break_d);
		
		
		// Calculate the expected distance and velocity the robot should have achieved by the time specified.
		//
		double expectedDistance = 0.0;
		double expectedVelocity = 0.0;
		double expectedAcceleration = 0.0;
		
		if (t <= 0.0)
		{
			expectedVelocity = vi;
			expectedDistance = 0.0;
			expectedAcceleration = 0.0;
		}
		else if (t <= accl_t)
		{
			expectedVelocity = getVelocity(t, accl_vi, maxAccl);
			expectedDistance = getDistance(t, accl_vi, maxAccl);
			expectedAcceleration = maxAccl;
		}
		else if (t <= (accl_t + cruise_t))
		{
			expectedAcceleration = 0.0;
			expectedVelocity = accl_vf;
			expectedDistance = accl_d + getDistance(t - accl_t, accl_vf, 0.0);
		}
		else if (t <= (accl_t + cruise_t + break_t))
		{
			expectedVelocity = getVelocity(t - (accl_t + cruise_t), break_vi, maxBreak);
			expectedDistance = accl_d + cruise_d + getDistance(t - (accl_t + cruise_t), break_vi, maxBreak);
			expectedAcceleration = maxBreak;
		}
		else
		{
			expectedVelocity = 0.0;
			expectedDistance = distance;
			expectedAcceleration = 0.0;
		}
		
		return new MotionPosition(bearing, expectedDistance, expectedVelocity, expectedAcceleration);
	}
	
	public static MotionPosition getExpectedRotationPosition(double t, double angle, double maxRAccl)
	{
		/**
		 * Maximum deceleration value.
		 */
		double maxRBreak = -maxRAccl;
				
				
		/**
		 * Distance traveled while accelerating.
		 */
		double accl_d = angle / 2;

		/**
		 * Time spent accelerating.
		 */
		double accl_t = getTimeToDistance(accl_d, 0.0, maxRAccl);

		/**
		 * Velocity at beginning of acceleration.
		 */
		double accl_vi = 0.0;
		
		/**
		 * Velocity at end of acceleration.
		 */
		double accl_vf = getVelocity(accl_t, 0.0, maxRAccl);
		
		
		/**
		 * Velocity at beginning of breaking.
		 */
		double break_vi = accl_vf;
		
		/**
		 * Velocity at end of breaking.
		 */
		double break_vf = 0.0;
		
		/**
		 * Distance traveled while breaking.
		 */
		double break_d = accl_d;				
		
		/**
		 * Time spent breaking.
		 */
		double break_t = accl_t;
		

	SmartDashboard.putNumber("accl_t", accl_t);
	SmartDashboard.putNumber("accl_d", accl_d);
	SmartDashboard.putNumber("break_t", break_t);
	SmartDashboard.putNumber("break_d", break_d);
		
		
		// Calculate the expected distance and velocity the robot should have achieved by the time specified.
		//
		double expectedRotation = 0.0;
		double expectedVelocity = 0.0;
		double expectedAcceleration = 0.0;
		
		if (t <= 0.0)
		{
			expectedVelocity = 0.0;
			expectedRotation = 0.0;
			expectedAcceleration = 0.0;
		}
		else if (t <= accl_t)
		{
			expectedVelocity = getVelocity(t, accl_vi, maxRAccl);
			expectedRotation = getDistance(t, accl_vi, maxRAccl);
			expectedAcceleration = maxRAccl;
		}
		else if (t <= (accl_t + break_t))
		{
			expectedVelocity = getVelocity(t - accl_t, break_vi, maxRBreak);
			expectedRotation = accl_d + getDistance(t - accl_t, break_vi, maxRBreak);
			expectedAcceleration = maxRBreak;
		}
		else
		{
			expectedVelocity = 0.0;
			expectedRotation = angle;
			expectedAcceleration = 0.0;
		}
		
		return new MotionPosition(expectedRotation, 0.0, expectedVelocity, expectedAcceleration);
	}	
	
	/***
	 * Return the time that the specified change in velocity occurred assuming the constant acceleration
	 * specified.
	 *
	 * If the return value is negative it  means that you are accelerating in the wrong direction
	 * to achieve the desired final velocity.
	 * 
	 * @param vf	Final velocity
	 * @param vi	Initial velocity
	 * @param a		Acceleration
	 * 
	 * @return		Time to achieve velocity at specified acceleration.
	 */
	public static double getTimeToVelocity(double vf, double vi, double a)
	{
		return (vf - vi) / a;
	}

	/***
	 * Return the time that a distance will be reached with the initial velocity and acceleration specified. 
	 * 
	 * If the time returned is negative it means the acceleration and or velocity are in the wrong direction 
	 * to achieve the desired distance.
	 * 
	 * If the distance will be achieved more than once with the specified conditions, the first (minimum) time to
	 * that distance is returned.
	 * 
	 * @param d		Distance desired.
	 * @param vi	Initial velocity.
	 * @param a		Acceleration
	 * 
	 * @return		The time necessary to achieve the specified distance.
	 */
	public static double getTimeToDistance(double d, double vi, double a)
	{
		double result = 0.0;

		double t1 = getPosQuadSolution(0.5 * a, vi, -d);
		double t2 = getNegQuadSolution(0.5 * a, vi, -d);
		
		if (t1 > 0.0 && t2 > 0.0)
		{
			result = Math.min(t1, t2);
		}
		else
		{
			result = Math.max(t1, t2);
		}
		
		return result;
	}
	
	/***
	 * Return the distance traveled in the specified amount of time with the specified initial velocity
	 * and acceleration.
	 * 
	 * d = vi * t + 1/2 * a * t^2
	 * 
	 * @param t		The time under acceleration.
	 * @param vi	The initial velocity.
	 * @param a		The acceleration.
	 * 
	 * @return		The distance traveled.
	 */
	public static double getDistance(double t, double vi, double a)
	{
		return (vi * t) + (0.5 * a * t * t);
	}
	
	/***
	 * Return the velocity achieved after the specified amount of time with the specified initial velocity
	 * and acceleration.
	 * 
	 * 
	 * @param t		The time under acceleration.
	 * @param vi	The initial velocity.
	 * @param a		The acceleration.
	 * 
	 * @return		The velocity achieved.
	 */
	public static double getVelocity(double t, double vi, double a)
	{
		return vi + (a * t);
	}
	
	/***
	 * 
	 * @param a
	 * @param b
	 * @param c
	 * 
	 * @return
	 */
	public static double getPosQuadSolution(double a, double b, double c)
	{
		return (-b + Math.sqrt((b * b) - (4.0 * a * c))) / (2.0 * a);
	}
	
	/***
	 * 
	 * @param a
	 * @param b
	 * @param c
	 * 
	 * @return
	 */
	public static double getNegQuadSolution(double a, double b, double c)
	{
		return (-b - Math.sqrt((b * b) - (4.0 * a * c))) / (2.0 * a);
	}

}
