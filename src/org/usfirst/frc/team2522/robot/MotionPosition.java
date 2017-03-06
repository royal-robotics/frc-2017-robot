package org.usfirst.frc.team2522.robot;

public class MotionPosition
{
	public double bearing;
	public double distance;
	public double velocity;
	public double acceleration;
	
	public MotionPosition(double bearing, double distance, double velocity, double acceleration)
	{
		this.bearing = bearing;
		this.distance = distance;
		this.velocity = velocity;
		this.acceleration = acceleration;
	}
}
