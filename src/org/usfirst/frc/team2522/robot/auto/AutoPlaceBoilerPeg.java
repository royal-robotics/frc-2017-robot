package org.usfirst.frc.team2522.robot.auto;

import org.usfirst.frc.team2522.robot.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * 
 *
 */
public class AutoPlaceBoilerPeg extends AutoRoutine
{
	public static double alignRotation = 0.0;

	/**
	 * 
	 */
	@Override
	public void Initialize(Robot robot)
	{
		super.Initialize(robot);
		
		alignRotation = 0.0;
		
		ImageUtils.setCamera(robot.cameraLow);
	}
	

	/**
	 * 
	 */
	public void Periodic(Robot robot) {
		if (autoStep == 0) 
		{
			robot.setShooterPower(0.75);
			if (AutonomousController.driveTo(robot, 65.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 1)
		{
			double angle = -60;
			
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
			alignRotation = ImageUtils.getPegRotationError(75.0);
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getPegRotationError(75.0);
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
			if (AutonomousController.driveTo(robot, 50.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 5) 
		{
			alignRotation = ImageUtils.getPegRotationError(30.0);
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getPegRotationError(30.0);
			}
			ImageUtils.setCamera(robot.cameraHigh);
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
			if (AutonomousController.driveTo(robot, 24.0, 150, 80.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 8) 
		{
			robot.gearDrapes.set(DoubleSolenoid.Value.kForward);
			try {
				Thread.sleep(350);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			autoStep++;
		}
		else if (autoStep == 9) 
		{
			robot.gearPushout.set(DoubleSolenoid.Value.kForward);
			try {
				Thread.sleep(250);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			autoStep++;
		}
		else if (autoStep == 10) 
		{
			if (AutonomousController.driveTo(robot, -5.0, 35.0, 50.0))
			{
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				autoStep++;
			}
		}
		else if (autoStep == 11) 
		{
			robot.gearPushout.set(DoubleSolenoid.Value.kReverse);
			
			if (AutonomousController.driveTo(robot, -30.0, 35.0, 35.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 12) 
		{
			if (AutonomousController.rotateTo(robot, -180, 300, 500))
			{
				autoStep++;
			}
		}
		else if (autoStep == 13) 
		{
			alignRotation = ImageUtils.getBoilerRotationError(120.0);
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getBoilerRotationError(120.0);
			}
			autoStep++;
		}
		else if (autoStep == 14)
		{
			if (AutonomousController.rotateTo(robot, alignRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 15) 
		{
			if (AutonomousController.driveTo(robot, 91.0))
			{
				robot.setFeederPower(0.75);
				autoStep++;
			}
		}
		else
		{
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}
	
}
