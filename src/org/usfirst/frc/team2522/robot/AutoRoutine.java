package org.usfirst.frc.team2522.robot;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public abstract class AutoRoutine
{
	protected int autoStep = 0;
	private DriverStation.Alliance autoAlliance = Alliance.Invalid;
	private List<AutoStep> steps = new ArrayList<AutoStep>();
	private List<String> stepNames = new ArrayList<String>();
	
	/**
	 * 
	 * @param robot
	 */
	public void Initialize(Robot robot)
	{
		autoStep = 0;
		autoAlliance = AutonomousController.getAlliance();
		
		if (this.autoStep < this.steps.size())
		{
			AutonomousController.println("Starting: " + this.getStepName());
		}
	}
	
	/**
	 * 
	 * @param robot
	 */
	public void Periodic(Robot robot)
	{
		if (this.autoStep < this.steps.size())
		{
			AutoStep step = this.steps.get(this.autoStep);
			
			if (step.run(robot))
			{
				AutonomousController.println("Finished" + this.getStepName());
				
				this.autoStep++;
				
				if (this.autoStep < this.steps.size())
				{
					AutonomousController.println("Starting: " + this.getStepName());
				}
				else
				{
					AutonomousController.println("Auto Steps Complete");
				}
			}
		}
		else
		{
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}
	
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
	public int getStepNumber()
	{
		return autoStep;
	}

	/**
	 * 
	 * @return
	 */
	public String getStepName()
	{
		String result = "Finished";
		
		if (this.autoStep < this.steps.size())
		{
			result = this.stepNames.get(this.getStepNumber());
		}
		
		return result;
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
	 * @param s
	 */
	public void addAutoStep(String name, AutoStep step)
	{
		this.steps.add(step);
	}	
}
