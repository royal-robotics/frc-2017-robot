package org.usfirst.frc.team2522.robot;

import org.usfirst.frc.team2522.robot.auto.*;

import java.util.Map;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/***
 * 
 *
 */
public final class AutonomousController 
{
	/**
	 * 
	 */
	public static boolean autoIsDriving = false;
	
	public static double autoDriveStartValue = 0.0;

	/**
	 * Keep track of time in auto mode.
	 */
	private static Timer autoTimer = new Timer();
	
	/**
	 * Stores the currently selected auto routine instance.
	 */
	private static AutoRoutine autoRoutine;
	
	/**
	 * Stores the currently active alliance.
	 * 
	 */
	public static DriverStation.Alliance autoAlliance;
	
	
	@SuppressWarnings("serial")
	private static final Map<Integer, Map<Integer, AutoRoutine>> autoFieldPosMap = new HashMap<Integer, Map<Integer, AutoRoutine>>() {{
				put(1, new HashMap<Integer, AutoRoutine>() {{
					put(1, new AutoPlaceBoilerPeg());
				}});
				put(2, new HashMap<Integer, AutoRoutine>() {{
					put(1, new AutoPlaceCenterPeg());
					put(2, new AutoPlaceCenterDeadRecon());
				}});
				put(3, new HashMap<Integer, AutoRoutine>() {{
					put(1, new AutoPlaceOutsidePeg());
				}});
			}};
						
	/**
	 * Returns the current alliance of the robot.
	 * 
	 * @return
	 */
	public static DriverStation.Alliance getAlliance()
	{
		return DriverStation.getInstance().getAlliance();
	}
	
	/**
	 * Return the result of the robot field start position selector.
	 *  
	 *  For 2017 Steamworks the positions are as follows:
	 *  
	 *  	1: Robot position back bumper touching drive station wall facing out, with outer edge of right bumper about 1/2" in from edge of polycarb wall opening.
	 *  	2: Robot position back bumper touching drive station wall facing out, centered on gear peg as good as possible.
	 *  	3: Robot position back bumper touching drive station wall facing out, with 
	 *  
	 * @return The id of the field starting position.
	 */
	public static int getFieldStartPosition(Robot robot)
	{
		int fieldValue = 0;
		fieldValue += robot.autoselectStick.getRawButton(2) ? 1 : 0;
		fieldValue += robot.autoselectStick.getRawButton(3) ? 2 : 0;
		fieldValue += robot.autoselectStick.getRawButton(4) ? 4 : 0;
		fieldValue += robot.autoselectStick.getRawButton(5) ? 8 : 0;
		return fieldValue + 1;
	}

	/**
	 * 
	 * 
	 * @return
	 */
	public static int getAutoRoutineId(Robot robot)
	{
		int autoValue = 0;
		autoValue += robot.autoselectStick.getRawButton(13) ? 1 : 0;
		autoValue += robot.autoselectStick.getRawButton(14) ? 2 : 0;
		autoValue += robot.autoselectStick.getRawButton(15) ? 4 : 0;
		autoValue += robot.autoselectStick.getRawButton(16) ? 8 : 0;
		return autoValue + 1;
	}
	
	/**
	 *  
	 * @return
	 */
	public static AutoRoutine getAutoRoutine(Robot robot)
	{
		AutoRoutine result = null;
		
		Map<Integer, AutoRoutine> autoRoutines = autoFieldPosMap.get(getFieldStartPosition(robot));
		if (autoRoutines != null)
		{
			result = autoRoutines.get(getAutoRoutineId(robot));
		}
		
		return result;
	}
	
	
	/**
	 * 
	 * 
	 * @return
	 */
	public static double getStartDelay()
	{
		// TODO: read the joystick access controlling the auto start delay.
		return 0.0;
	}
	
	/**
	 * 
	 * @param robott
	 */
	public static void Initialize(Robot robot)
	{
		autoTimer.reset();
		autoTimer.start();

		autoIsDriving = false;
		
		autoRoutine = getAutoRoutine(robot);
		if(autoRoutine != null)
			autoRoutine.Initialize(robot);
	}
	
	/**
	 * 
	 * @param robot
	 */
	public static void Periodic(Robot robot)
	{
		if (autoRoutine != null)
		{
			autoRoutine.Periodic(robot);
		}
	}
	
	
	
	public static boolean rotateTo(Robot robot, double angle)
	{
		return rotateTo(robot, angle, 200.0, 300.0);
	}
	
	public static boolean rotateTo(Robot robot, double angle, double maxVel, double maxAcc)
	{
		if (Math.abs(angle) > 1.0)
		{
			if (!autoIsDriving)
			{
				SmartDashboard.putNumber("Auto-Rotate-Desired", angle);
	System.out.println("AutonomousController.rotateTo(): Requested Rotation=" + angle);			
				autoDriveStartValue = robot.getCurrentBearing();
				
				robot.driveController.rotate(angle, maxVel, maxAcc);
				autoIsDriving = true;
			}
			else
			{
				autoIsDriving = (robot.driveController.motionStartTime != 0.0);
			}
			
			if (!autoIsDriving)
			{
				SmartDashboard.putNumber("Auto-Rotate-Actual", robot.getCurrentBearing() - autoDriveStartValue);
	System.out.println("AutonomousController.rotateTo(): Actual Rotation=" + angle);			
			}
			
			return !autoIsDriving;
		}
		else
		{
			return true;
		}
	}

	public static boolean driveTo(Robot robot, double distance)
	{
		return driveTo(robot, distance, 150, 100);
	}
	
	public static boolean driveTo(Robot robot, double distance, double maxVel, double maxAcc)
	{
		if (Math.abs(distance) > 0.5)
		{
			if (!autoIsDriving)
			{
				autoDriveStartValue = robot.leftDriveEncoder.getDistance();
	
				robot.driveController.drive(distance, maxVel, maxAcc);
				autoIsDriving = true;
			}
			else
			{
				autoIsDriving = (robot.driveController.motionStartTime != 0.0);
			}
			
			if (!autoIsDriving)
			{
				SmartDashboard.putNumber("Auto-Drive", robot.leftDriveEncoder.getDistance() - autoDriveStartValue);
			}
			
			return !autoIsDriving;
		}
		else
		{
			return true;
		}
	}
	
}
