package org.usfirst.frc.team2522.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image and
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
  public class cameras extends IterativeRobot {
	Thread visionThread;
	boolean cameraButtonHeld = false;
	boolean camera1on = false;
	
	@Override
	public void robotInit() {
		
		visionThread = new Thread(() -> {
			UsbCamera camera = new UsbCamera("camera", 0);
			UsbCamera camera2 = new UsbCamera("camera2", 1);
			CameraServer.getInstance().addCamera(camera);
//			CameraServer.getInstance().addCamera(camera2);
//			MjpegServer server = new MjpegServer("camera", 0);
			MjpegServer server = CameraServer.getInstance().addServer("camera");
//			CvSink server = new CvSink("camera2");
			server.setSource(camera);
//			CvSink image = CameraServer.getInstance().getVideo(camera);
//			CvSink image2 = CameraServer.getInstance().getVideo(camera2);
//			server.setEnabled(true);
//			CameraServer.getInstance().addServer(server);
			Joystick stick = new Joystick (0);

//			// This cannot be 'true'. The program will never exit if it is. This
//			// lets the robot stop this thread when restarting robot code or
//			// deploying.

			while (!Thread.interrupted()) {
				if (stick.getRawButton(5) && (!cameraButtonHeld))
				{
					cameraButtonHeld = true;
					if (camera1on)
					{
						server.setSource(camera2);
						camera1on = false;
					}
					else
					{
						server.setSource(camera);
						camera1on = true;
					}
				}
				else if (!stick.getRawButton(5) && (cameraButtonHeld))
				{
					cameraButtonHeld = false;
				}
				
			}
			});

		visionThread.setDaemon(true);
		visionThread.start();
	    }
	}