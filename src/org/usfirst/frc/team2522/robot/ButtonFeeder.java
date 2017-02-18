package org.usfirst.frc.team2522.robot;

import java.util.*;


public enum ButtonFeeder {
	INSTANCE; //Singleton Pattern
	
	private List<Button> buttons = new ArrayList<Button>();
	
	public void addButton(Button button) {
		buttons.add(button);
	}
	
	public void feed() {
		for(Button button : buttons) {
			button.feed();
		}
	}
}