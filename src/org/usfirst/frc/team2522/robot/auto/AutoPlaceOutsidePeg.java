package org.usfirst.frc.team2522.robot.auto;

import org.usfirst.frc.team2522.robot.AutoRoutine;
import org.usfirst.frc.team2522.robot.AutonomousController;
import org.usfirst.frc.team2522.robot.ImageUtils;
import org.usfirst.frc.team2522.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPlaceOutsidePeg extends AutoRoutine
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
	


	@Override
	public void Periodic(Robot robot) {
		if (autoStep == 0) 
		{
			if (AutonomousController.driveTo(robot, 78.0))	// practice 72
			{
				autoStep++;
			}
		}
		else if (autoStep == 1)
		{
			double angle = 65;							// practice 60
			
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
			alignRotation = ImageUtils.getPegRotationError(75.0, "AutoImageStep" + autoStep + "_");
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getPegRotationError(75.0);
			}
			SmartDashboard.putNumber("Align Rotation", alignRotation);
			autoStep++;
		}
		else if (autoStep == 3)
		{
			if (AutonomousController.rotateTo(robot, alignRotation, 300.0, 500.0))
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
			alignRotation = ImageUtils.getPegRotationError(30.0, "AutoImageStep" + autoStep + "_");
			while(alignRotation == Double.NaN)
			{
				alignRotation = ImageUtils.getPegRotationError(30.0);
			}
			autoStep++;
		}
		else if (autoStep == 6)
		{
			if (AutonomousController.rotateTo(robot, alignRotation, 300.0, 500.0))
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
		else
		{
			robot.shifter.set(DoubleSolenoid.Value.kForward); // high gear
			robot.myDrive.tankDrive(0.0, 0.0);
		}
	}

}
