package org.usfirst.frc.team2522.robot.auto;

import org.usfirst.frc.team2522.robot.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoPlaceCenterPeg extends AutoRoutine
{
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
		
		ImageUtils.setCamera(robot.cameraLow);
	}

	/**
	 * 
	 */
	public void Periodic(Robot robot) {
		if (autoStep == 0)
		{
			if (AutonomousController.driveTo(robot, 24.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 1)
		{
			ImageUtils.setCamera(robot.cameraLow);
			alignRotation = ImageUtils.getPegRotationError(60.0, "AutoImageStep" + autoStep + "_");
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getPegRotationError(60.0);
			}
			autoStep++;
		}
		else if (autoStep == 2)
		{
			if (AutonomousController.rotateTo(robot, alignRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 3) 
		{
			if (AutonomousController.driveTo(robot, 33.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 4)
		{
			ImageUtils.setCamera(robot.cameraLow);
			alignRotation = ImageUtils.getPegRotationError(60.0, "AutoImageStep" + autoStep + "_");
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getPegRotationError(60.0);
			}
			autoStep++;
		}
		else if (autoStep == 5)
		{
			if (AutonomousController.rotateTo(robot, alignRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 6) 
		{
			if (AutonomousController.driveTo(robot, 27.0, 150, 80.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 7) 
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
		else if (autoStep == 8) 
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
		else if (autoStep == 9) 
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
		else if (autoStep == 10) 
		{
			robot.gearPushout.set(DoubleSolenoid.Value.kReverse);

			if (AutonomousController.driveTo(robot, -30.0, 35.0, 35.0))
			{
				autoStep++;
			}
		}
		else
		{
			robot.shifter.set(DoubleSolenoid.Value.kForward); // high gear
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}

}
