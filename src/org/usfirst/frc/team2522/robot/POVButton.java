package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.*;

public class POVButton extends Button
{
	private final int povPos;

	/***
	 * Create a POV button to treat POV positions like buttons on a joystick.
	 * 
	 * @param stick 	The Joystick that the POV is on.
	 * @param buttonId	The Id/Index of the POV button. 
	 * @param povPos	The position indicator value to use 0, 45, 90, 135, 180, 225, 270, 315
	 * @param type		The type of button to create.
	 */
	public POVButton(Joystick stick, int buttonId, int povPos, ButtonType type)
	{
		super(stick, buttonId, type);
		this.povPos = povPos;
	}
	
	/***
	 * Get the position of the button.
	 * 
	 * @return true if the button is pressed and false otherwise.
	 */
	protected boolean getPosition()
	{
		return this.stick.getPOV(this.buttonId) == this.povPos;
	}
}
