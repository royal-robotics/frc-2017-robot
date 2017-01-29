package org.usfirst.frc.team2522.robot;

import java.nio.ByteBuffer;

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

//import com.ni.vision.NIVision.Image;
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
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
			
			VideoCapture camera = new VideoCapture(1);
//			camera.set(org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE, 100);
			//camera.set(org.opencv.videoio.Videoio.CAP_PROP_BRIGHTNESS, 100);
			//camera.set(org.opencv.videoio.Videoio.CAP_PROP_AUTO_EXPOSURE, 0);
			//camera.set(org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE, 100);
//			camera.set(org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE, 100);
			//camera.set(org.opencv.videoio.Videoio.CAP_PROP_AUTO_EXPOSURE, 1);
			//camera.set(org.opencv.videoio.Videoio.CAP_PROP_AUTO_EXPOSURE, 1);
			
			//camera.set(org.opencv.videoio.Videoio.CAP_PROP_SETTINGS, 1);
			SmartDashboard.putDouble("PROP_EXPOSURE", -100);
			SmartDashboard.putDouble("PROP_BRIGHTNESS", 0);
			
			Mat mat = new Mat();
			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
			    //byte[] bytes = new byte[] {Byte.MIN_VALUE, Byte.MIN_VALUE, Byte.MIN_VALUE, Byte.MIN_VALUE, Byte.MIN_VALUE, Byte.MIN_VALUE, Byte.MIN_VALUE, Byte.MIN_VALUE };
			    
				//camera.set(org.opencv.videoio.Videoio.CAP_PROP_AUTO_EXPOSURE, ByteBuffer.wrap(bytes).getDouble());
//			    camera.set(org.opencv.videoio.Videoio.CAP_PROP_AUTO_EXPOSURE, 0);
//				camera.set(org.opencv.videoio.Videoio.CAP_PROP_AUTOFOCUS, 0);
				camera.set(org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE, SmartDashboard.getDouble("PROP_EXPOSURE"));
				camera.set(org.opencv.videoio.Videoio.CAP_PROP_BRIGHTNESS, SmartDashboard.getDouble("PROP_BRIGHTNESS"));
//				camera.set(org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE, 0);
//				camera.set(org.opencv.videoio.Videoio.CAP_PROP_EXPOSURE, 0);
				
		        SmartDashboard.putBoolean("button 1", stick.getRawButton(1));
				camera.read(mat);
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}
	    }
