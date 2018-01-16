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
	public double alignRotation = 0.0;

	/**
	 * 
	 */
	@Override
	public void Initialize(Robot robot)
	{
		super.Initialize(robot);
		
		alignRotation = 0.0;
		
		ImageUtils.setCamera(robot.cameraHigh);
		
		this.addAutoStep("Put Hood Down", (Robot r) -> {
	    	robot.shooterHood.set(DoubleSolenoid.Value.kReverse);		// hood down				
			return true;
		});
		
		this.addAutoStep("Start Shooter", (Robot r) -> {
			robot.setShooterPower(0.60);
			return true;
		});
		
		this.addAutoStep("Drive Forward", (Robot r) -> {			
			if (AutonomousController.driveTo(robot, -64.0))
			{
				return true;
			}
			
			return false;
		});
		
		this.addAutoStep("Turn Towards Boiler", (Robot r) -> {
			double angle = -35.0;
			
			if (this.getAlliance() == DriverStation.Alliance.Blue)
			{
				angle = -angle;
			}
				
			if (AutonomousController.rotateTo(robot, angle, 300.0, 500.0))
			{
				return true;
			}
			
			return false;
		});
		
		this.addAutoStep("Get Boiler Rotation Error", (Robot r) -> {
			alignRotation = ImageUtils.getBoilerRotationError(57.0, "AutoImageStep" + autoStep + "_");
			
			if (alignRotation != Double.NaN)
			{
				AutonomousController.println("  AutoRotation Error = " + alignRotation);
				return true;
			}
			
			return false;
		});
		
		this.addAutoStep("Turn Towards Boiler", (Robot r) -> {
			if (AutonomousController.rotateTo(robot, alignRotation))
			{
				return true;
			}
			
			return false;
		});
		
		this.addAutoStep("Drive to Boiler Wall", (Robot r) -> {
			if (AutonomousController.driveTo(robot, 67.0))	// 57
			{
				return true;
			}
			
			return false;
		});
		
		this.addAutoStep("Start Feeder and Intake", (Robot r) -> {
			robot.myDrive.tankDrive(0.0, 0.0);
			robot.setFeederPower(robot.getDashboardFeederPower());
			robot.unjammer.set(robot.getDashboardUnjammerPower());
			robot.intakeSolenoid.set(DoubleSolenoid.Value.kForward);
			robot.intakeTalon.set(1.0);
			return true;
		});
	}

}
