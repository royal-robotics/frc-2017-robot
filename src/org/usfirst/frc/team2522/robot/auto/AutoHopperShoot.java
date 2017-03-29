package org.usfirst.frc.team2522.robot.auto;

import org.usfirst.frc.team2522.robot.AutoRoutine;
import org.usfirst.frc.team2522.robot.AutonomousController;
import org.usfirst.frc.team2522.robot.ImageUtils;
import org.usfirst.frc.team2522.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoHopperShoot extends AutoRoutine {

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
		robot.setShooterPower(0.75);	// .42
		
		if (autoStep == 0) 
		{
			if (robot.practiceBot)
			{
		    	robot.shooterHood.set(DoubleSolenoid.Value.kReverse);		// hood down				
			}
			else
			{
		    	robot.shooterHood.set(DoubleSolenoid.Value.kForward);		// hood up				
			}
			
			if (AutonomousController.driveTo(robot, -65.0))		// -70
			{
				autoStep++;
			}
		}
		else if (autoStep == 1)
		{
			double angle = 55.0;		// 45
			
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
			if (AutonomousController.driveTo(robot, -60.0))  // was -52
			{
				autoStep++;
			}
		}
		else if (autoStep == 3)
		{
			double angle = -75.0;	// -65.0

			if (this.getAlliance() == DriverStation.Alliance.Blue)
			{
				angle = -angle;
			}
				
			if (AutonomousController.rotateTo(robot, angle, 300.0, 500.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 4) 
		{
			if (AutonomousController.driveTo(robot, 40.0)) // 24.0
			{
				autoStep++;
			}
		}
		else if (autoStep == 5) 
		{
			alignRotation = ImageUtils.getBoilerRotationError(80.0, "AutoImageStep" + autoStep + "_");
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getBoilerRotationError(80.0);
			}
			autoStep++;
		}
		else if (autoStep == 6)
		{
			if (AutonomousController.rotateTo(robot, alignRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 7)
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
