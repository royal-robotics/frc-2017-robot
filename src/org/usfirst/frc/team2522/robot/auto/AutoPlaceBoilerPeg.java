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
	public static double pegAlignRotation = 0.0;

	/**
	 * 
	 */
	@Override
	public void Initialize(Robot robot)
	{
		super.Initialize(robot);
		
		ImageUtils.setCamera(robot.cameraLow);
	}
	

	/**
	 * 
	 */
	public void Periodic(Robot robot) {
		if (autoStep == 0) 
		{
			if (AutonomousController.driveTo(robot, 68.0))
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
				
			if (AutonomousController.rotateTo(robot, -60.0, 200, 450))
			{
				autoStep++;
			}
		}
		else if (autoStep == 2) 
		{
			pegAlignRotation = ImageUtils.getPegRotationError();
			while(pegAlignRotation == -999.0)
			{
				pegAlignRotation = ImageUtils.getPegRotationError();
			}
			autoStep++;
		}
		else if (autoStep == 3)
		{
			if (AutonomousController.rotateTo(robot, pegAlignRotation))
			{
				autoStep++;
			}
		}
		else if (autoStep == 4) 
		{
			if (AutonomousController.driveTo(robot, 60.0))
			{
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				autoStep++;
			}
		}
		else if (autoStep == 5) 
		{
			if (AutonomousController.driveTo(robot, 20.0, 150, 80.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 6) 
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
		else if (autoStep == 7) 
		{
			robot.gearPushout.set(DoubleSolenoid.Value.kForward);
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			autoStep++;
		}
		else if (autoStep == 8) 
		{
			if (AutonomousController.driveTo(robot, -5.0, 35.0, 50.0))
			{
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				autoStep++;
			}
		}
		else if (autoStep == 9) 
		{
			robot.gearPushout.set(DoubleSolenoid.Value.kReverse);
			
			if (AutonomousController.driveTo(robot, -30.0, 35.0, 35.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 10) 
		{
			if (AutonomousController.rotateTo(robot, 180, 200, 450))
			{
				autoStep++;
			}
		}
//		else if (autoStep == 11) 
//		{
//			if (AutonomousController.driveTo(robot, 80.0))
//			{
//				autoStep++;
//			}
//		}
		else
		{
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}
	
}
