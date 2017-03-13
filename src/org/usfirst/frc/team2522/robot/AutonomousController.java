package org.usfirst.frc.team2522.robot;

import org.usfirst.frc.team2522.robot.auto.*;

import java.util.Map;
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
	
	/**
	 * 
	 */
	@SuppressWarnings("serial")
	private static final Map<Integer, Map<Integer, AutoRoutine>> autoFieldPosMap = new HashMap<Integer, Map<Integer, AutoRoutine>>() {{
				put(1, new HashMap<Integer, AutoRoutine>() {{
					put(1, new AutoPlaceBoilerPeg());
				}});
				put(2, new HashMap<Integer, AutoRoutine>() {{
					put(1, new AutoPlaceCenterPeg());
				}});
				put(2, new HashMap<Integer, AutoRoutine>() {{
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
	public static int getFieldStartPosition()
	{
		// TODO: read field select switch from drive station.
		return 1;
	}

	/**
	 * 
	 * 
	 * @return
	 */
	public static int getAutoRoutineId()
	{
		// TODO: read auto routine select switch from drive station.
		return 1;
	}
	
	/**
	 *  
	 * @return
	 */
	public static AutoRoutine getAutoRoutine()
	{
		AutoRoutine result = null;
		
		Map<Integer, AutoRoutine> autoRoutines = autoFieldPosMap.get(getFieldStartPosition());
		if (autoRoutines != null)
		{
			result = autoRoutines.get(getAutoRoutineId());
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
	 * @param robot
	 */
	public static void Initialize(Robot robot)
	{
		autoTimer.reset();
		autoTimer.start();

		autoIsDriving = false;
		
		autoRoutine = getAutoRoutine();
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
		if (!autoIsDriving)
		{
			robot.driveController.rotate(angle, maxVel, maxAcc);
			autoIsDriving = true;
		}
		else
		{
			autoIsDriving = (robot.driveController.motionStartTime != 0.0);
		}
		
		return !autoIsDriving;
	}

	public static boolean driveTo(Robot robot, double distance)
	{
		return driveTo(robot, distance, 150, 100);
	}
	
	public static boolean driveTo(Robot robot, double distance, double maxVel, double maxAcc)
	{
		if (!autoIsDriving)
		{
			robot.driveController.drive(distance, maxVel, maxAcc);
			autoIsDriving = true;
		}
		else
		{
			autoIsDriving = (robot.driveController.motionStartTime != 0.0);
		}
		
		return !autoIsDriving;
	}
	
}
