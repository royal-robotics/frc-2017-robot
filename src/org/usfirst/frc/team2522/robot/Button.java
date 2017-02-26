package org.usfirst.frc.team2522.robot;

import edu.wpi.first.wpilibj.*;

public class Button {
	enum ButtonType {
		Hold, //isPressed function returns true while the button is down
		Toggle //isPressed function returns true a single time until button is toggled up and back down
	}
	
	private final Joystick stick;
	private final int buttonId;
	private final ButtonType type;
	private boolean previousValue;
	
	public Button(Joystick stick, int buttonId, ButtonType type) {
		this.stick = stick;
		this.buttonId = buttonId;
		this.type = type;
		this.previousValue = stick.getRawButton(buttonId);
		
		//ButtonFeeder.INSTANCE.addButton(this);
	}
	
	public void feed() {
		//previousValue = stick.getRawButton(buttonId);
	}
	
	public boolean isPressed() {
		boolean value = stick.getRawButton(buttonId);
		boolean retValue = false;
		switch(type) {
			case Hold:
				retValue = value;
				break;
			case Toggle:
			{
				retValue = !previousValue && value;
				break;
			}
		}
		previousValue = value;
		return retValue;
	}
}
