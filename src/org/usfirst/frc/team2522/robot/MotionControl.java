package org.usfirst.frc.team2522.robot;

public class MotionControl 
{
	/**
	 * 
	 * @param t
	 * @param distance
	 * @param bearing
	 * @param vi
	 * @param maxVel
	 * @param maxAcc
	 * 
	 * @return
	 */
	public static MotionPosition GetExpectedPosition(double t, double distance, double bearing, double vi, double maxVel, double maxAcc)
	{
		// Calculate the time it takes to get to the cruise speed (max velocity).
		double cruise_t = getTimeToVelocity(maxVel, vi, maxAcc);
		
		// Calculate distance to cruise speed (max velocity).
		double cruise_d = getDistance(cruise_t, vi, maxAcc);
		
		// Calculate the turn around distance as if there was no max velocity
		double turn_around = distance / 2.0;
		
		// Calculate the trajectory variables
		//
		double ta, da;	// Time and Distance spent accelerating
		double tc, dc;	// Time and Distance spent cruising
		double td, dd;	// Time and Distance spent deceleration
		
		if (Math.abs(cruise_d) >= Math.abs(turn_around))
		{
			da = turn_around;
			dc = 0.0;
			dd = turn_around;

			ta = getTimeToDistance(turn_around, vi, maxAcc);
			tc = 0.0;
			td = getTimeToDistance(turn_around, getVelocity(ta, vi, maxAcc), -maxAcc);
		}
		else
		{
			da = cruise_d;
			dc = distance - (2.0 * cruise_d);
			dd = cruise_d;

			ta = cruise_t;
			tc = getTimeToDistance(distance - (da + dd), maxVel, 0.0);
			td = cruise_t;
		}

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
		else if (t <= ta)
		{
			expectedVelocity = getVelocity(t, vi, maxAcc);
			expectedDistance = getDistance(t, vi, maxAcc);
			expectedAcceleration = maxAcc;
		}
		else if (t <= (ta + tc))
		{
			expectedVelocity = maxVel;
			expectedDistance = da + getDistance(t - ta, maxVel, 0.0);
			expectedAcceleration = 0.0;
		}
		else if (t <= (ta + tc + td))
		{
			expectedVelocity = getVelocity(t - (ta + tc), maxVel, -maxAcc);
			expectedDistance = da + dc + getDistance(t - (ta + tc), maxVel, -maxAcc);
			expectedAcceleration = -maxAcc;
		}
		else
		{
			expectedVelocity = 0.0;
			expectedDistance = distance;
			expectedAcceleration = 0.0;
		}
		
		return new MotionPosition(bearing, expectedDistance, expectedVelocity, expectedAcceleration);
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
