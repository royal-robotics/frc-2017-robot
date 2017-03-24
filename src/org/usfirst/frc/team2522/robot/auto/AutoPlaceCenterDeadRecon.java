package org.usfirst.frc.team2522.robot.auto;

import org.usfirst.frc.team2522.robot.AutoRoutine;
import org.usfirst.frc.team2522.robot.AutonomousController;
import org.usfirst.frc.team2522.robot.ImageUtils;
import org.usfirst.frc.team2522.robot.Robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPlaceCenterDeadRecon extends AutoRoutine {

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
			if (AutonomousController.driveTo(robot, 57.0))	// 33.0
			{
				autoStep++;
			}
		}
		else if (autoStep == 1) 
		{
			if (AutonomousController.driveTo(robot, 27.0, 150, 80.0))
			{
				autoStep++;
			}
		}
		else if (autoStep == 2) 
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
		else if (autoStep == 3) 
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
		else if (autoStep == 4) 
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
		else if (autoStep == 5) 
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
