package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public abstract class AutoRoutine
{
	protected int autoStep = 0;
	private DriverStation.Alliance autoAlliance = Alliance.Invalid;
	
	
	/**
	 * 
	 * @return
	 */
	public String getName()
	{
		String name = this.getClass().getName();
		return name.substring(name.lastIndexOf('.') + 1);
	}
	
	/**
	 * 
	 * @return
	 */
	public DriverStation.Alliance getAlliance()
	{
		return this.autoAlliance;
	}

	/**
	 * 
	 * @return
	 */
	public int getAutoStep()
	{
		return autoStep;
	}
	
	/**
	 * 
	 * @param robot
	 */
	public void Initialize(Robot robot)
	{
		autoStep = 0;
		autoAlliance = AutonomousController.getAlliance();
	}
	
	/**
	 * 
	 * @param robot
	 */
	public abstract void Periodic(Robot robot);
}
