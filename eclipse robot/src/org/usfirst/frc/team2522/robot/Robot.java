package org.usfirst.frc.team2522.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image and
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
public class Robot extends IterativeRobot {
	Thread visionThread;

	@Override
	public void robotInit() {
		visionThread = new Thread(() -> {
			Joystick stick = new Joystick (0);
			// Get the UsbCamera from CameraServer
			UsbCamera usbcamera = new UsbCamera("cam0", 0);
			UsbCamera usbcamera2 = new UsbCamera("cam1", 1);
			CameraServer.getInstance().startAutomaticCapture(usbcamera2);
			//UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture("cam1", 1);
			boolean previousButton = stick.getRawButton(1);
			int prevExp1 = 45;
			int prevExp2 = 0;
			int newExp;
					
			// Set the resolution
			usbcamera.setResolution(640, 480);
			usbcamera.setExposureManual(45);
			usbcamera2.setExposureManual(0);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				//cvSink = CameraServer.getInstance().getVideo(1);
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
		        SmartDashboard.putBoolean("button 1", stick.getRawButton(1));
		        SmartDashboard.putBoolean("button 2", stick.getRawButton(2));
		        SmartDashboard.putBoolean("button 3", stick.getRawButton(3));
		        SmartDashboard.putBoolean("button 4", stick.getRawButton(4));
		        newExp = SmartDashboard.getInt("Exposure 1", 45);
		        if (newExp != prevExp1) {
				   usbcamera.setExposureManual(newExp);
				   prevExp1 = newExp;
		        }
		        newExp = SmartDashboard.getInt("Exposure 2", 0);
		        if (newExp != prevExp2) {
		        	usbcamera2.setExposureManual (newExp);
		        	prevExp2 = newExp;
		        }
		        if (stick.getRawButton(1) != previousButton){
		        	//cvSink = null;
		        	if (stick.getRawButton(1)){		        	
		        	cvSink.setSource(usbcamera2);
		        	}
		        	else{
		        	cvSink.setSource(usbcamera);
		        	}
					//cvSink = CameraServer.getInstance().getVideo();
		        	previousButton = stick.getRawButton(1);
		        	}
				// Put a rectangle on the image
				if (stick.getRawButton(1)){
				Imgproc.rectangle(mat, new Point(100, 100), new Point(300, 200),
						new Scalar(255, 255, 255), 5);
				}
				else {Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
						new Scalar(255, 255, 255), 5);
				}
				
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}
	}
