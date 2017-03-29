package org.usfirst.frc.team2522.robot.auto;

import org.usfirst.frc.team2522.robot.AutoRoutine;
import org.usfirst.frc.team2522.robot.AutonomousController;
import org.usfirst.frc.team2522.robot.ImageUtils;
import org.usfirst.frc.team2522.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoBoilerShoot extends AutoRoutine {

	/**
	 * 
	 */
	public static double alignRotation = 0.0;

	/**
	 * 
	 */
	@Override
	public void Initialize(Robot robot)
	{
		super.Initialize(robot);
		
		alignRotation = 0.0;
		
		ImageUtils.setCamera(robot.cameraHigh);
	}
	


	/**
	 * 
	 */
	@Override
	public void Periodic(Robot robot) 
	{
		robot.setShooterPower(0.60);
		
		if (autoStep == 0) 
		{
	    	robot.shooterHood.set(DoubleSolenoid.Value.kReverse);		// hood down				
			
			if (AutonomousController.driveTo(robot, -64.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 1)
		{
			double angle = -35.0;
			
			if (this.getAlliance() == DriverStation.Alliance.Blue)
			{
				angle = -angle;
			}
				
			if (AutonomousController.rotateTo(robot, angle, 300.0, 500.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 2) 
		{
			alignRotation = ImageUtils.getBoilerRotationError(57.0, "AutoImageStep" + autoStep + "_");
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getBoilerRotationError(57.0);
			}
			autoStep++;
		}
		else if (autoStep == 3)
		{
			if (AutonomousController.rotateTo(robot, alignRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 4) 
		{
	    	robot.shooterHood.set(DoubleSolenoid.Value.kReverse);		// hood down				
			
			if (AutonomousController.driveTo(robot, 67.0))	//57
			{
				autoStep++;
			}
		}
		else if (autoStep == 5)
		{
			robot.setFeederPower(robot.getDashboardFeederPower());
			robot.unjammer.set(robot.getDashboardUnjammerPower());
			robot.intakeSolenoid.set(DoubleSolenoid.Value.kForward);
			robot.intakeTalon.set(1.0);
			autoStep++;
		}
		else
		{
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}

}
