package org.usfirst.frc.team2522.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ImageUtils
{
	public static final int	imageWidth = 640;
	public static final int imageHeight = 480;
	
	public static UsbCamera camera = null;
	public static CvSink cameraSink = null;
	public static CvSource cameraStream = null;
	
	public static Mat src_image = new Mat();
	public static Mat hsv_image = new Mat();
	public static Mat msk_image = new Mat();
	public static Mat hierarchy = new Mat();
	
	public static Scalar hsvLowerBounds = new Scalar(45, 100, 100, 0);		// yellow green
	public static Scalar hsvUpperBounds = new Scalar(75, 255, 255, 255);	// green cyan
	
	
	public static UsbCamera getCamera(int dev, String name)
	{
		UsbCamera camera = new UsbCamera(name, dev);
		
    	if (camera != null)
    	{
    		camera.setResolution(imageWidth, imageHeight);			
	    	
    		if (!camera.isConnected())
	    	{
	        	System.out.println(name + " Camera Not Connected.");
	        	camera.free();
	    	}
    		else
    		{
    			CameraServer.getInstance().addCamera(camera);
    		}
    	}
		
    	return camera;
	}
	
	public static synchronized void setCamera(UsbCamera camera)
	{
		if (ImageUtils.cameraStream == null)
		{
			ImageUtils.cameraStream = CameraServer.getInstance().putVideo("camera", imageWidth, imageHeight);
		}
		
		if ((ImageUtils.camera == null) || (ImageUtils.camera.getName() != camera.getName()))
		{
			if (ImageUtils.cameraSink != null)
			{
				CameraServer.getInstance().removeServer("opencv_" + ImageUtils.camera.getName());
				ImageUtils.cameraSink.free();
				ImageUtils.cameraSink = null;
			}

			if (camera != null)
			{
				ImageUtils.cameraSink = CameraServer.getInstance().getVideo(camera);
			}
		}

		ImageUtils.camera = camera;
	}
	
	public static List<Rect> processFrame(double minAspectRatio, double maxAspectRatio, boolean showImageBlobs, boolean showImageTargets, boolean saveImage)
	{
		List<Rect> rectangles = new ArrayList<Rect>();

		if (ImageUtils.cameraSink != null)
		{
			if (ImageUtils.cameraSink.grabFrame(src_image) == 0)
			{
				ImageUtils.cameraStream.notifyError(ImageUtils.cameraSink.getError());
				System.out.println(ImageUtils.cameraSink.getError());
			}
			else
			{
				rectangles = ImageUtils.getRectangles(src_image, minAspectRatio, maxAspectRatio);
				
				if (saveImage)
				{
					ImageUtils.saveRectangles(rectangles);
				}
				
				if (showImageBlobs)
				{
					ImageUtils.cameraStream.putFrame(ImageUtils.getMask(src_image));
				}
				else
				{
					if (showImageTargets)
					{
						String s = "";
						for(int index = 0; index < rectangles.size(); index++)
						{
							Rect r = rectangles.get(index);
							Imgproc.rectangle(src_image, new Point(r.x, r.y), new Point(r.x+r.width, r.y+r.height), new Scalar(0, 0, 255), 3);
	
							s = s + "{" + r.x + "," + r.y + "," + r.width + "," + r.height+"}";
						}
	
						SmartDashboard.putString("Image-Rects", s);
					}
					
					ImageUtils.cameraStream.putFrame(src_image);
				}
			}
		}
		
		return rectangles;
	}
	
	public static Mat getMask(Mat src_image)
	{
		Imgproc.cvtColor(src_image, hsv_image, Imgproc.COLOR_RGB2HSV_FULL);
		Core.inRange(hsv_image, hsvLowerBounds, hsvUpperBounds, msk_image);
		
		return msk_image;
	}
	
	public static List<Rect> getRectangles(Mat src_image, double minAspectRatio, double maxAspectRation)
	{
		ArrayList<Rect> results = new ArrayList<Rect>();
		
		msk_image = getMask(src_image);
		
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(msk_image, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		
		for(int i = 0; i < contours.size(); i++)
		{
			Rect r = Imgproc.boundingRect(contours.get(i));

			if ((r.width > 4) && (r.height > 4))
			{
				double rAspectRation = (double)r.height / (double)r.width;
				if ((rAspectRation >= minAspectRatio) && (rAspectRation <= maxAspectRation))
				{
					results.add(r);
				}
			}
		}
		
		return results;
	}
	
	public static double getPegTargetDistance(List<Rect> rectangles)
	{
		double result = -1.0;
		
		if ((rectangles.size() == 2) && (Math.abs(rectangles.get(0).y - rectangles.get(1).y) < 4))
		{
			// TOOD: calculate ratio of the height of the targets to distance from target.
			
			result = Math.max(rectangles.get(0).height, rectangles.get(1).height) * 
		}
		
		return result;
	}
	
	public static double getPegTargetBearing(List<Rect> rectangles)
	{
		double result = -1.0;
		
		// TODO: do triangle math with distance and pixels off center to find bearing correction.
		
		return result;
	}
	
	public static double getBoilerTargetDistance(List<Rect> rectangles)
	{
		double result = -1.0;
		
		// TODO: calculate ratio size to range. 
		
		return result;
	}
	
	public static double getBoilerTargetBearing(List<Rect> rectangles)
	{
		double result = -1.0;
		
		return result;
	}
	
	public static void saveRectangles(List<Rect> rectangles)
	{
		PrintStream ps = null;
		
		try
		{
			File f = new File("/home/lvuser/ImageRects0.txt");
			for(int i = 0; f.exists(); i++)
			{
				f = new File("/home/lvuser/ImageRects" + i + ".txt");
			}

			System.out.println("Image Rects File Created: " + f.getName());
			
			ps = new PrintStream(f);
		}
		catch(IOException e)
		{
			ps = null;
			e.printStackTrace();
		}
		
		if (ps != null)
		{
			for(int i = 0; i < rectangles.size(); i++)
			{
				Rect r = rectangles.get(i);
				ps.println(""+r.x +"\t"+r.y +"\t"+r.width +"\t"+r.height);
			}
			
			ps.close();
		}
	}
	
	
	public static MatOfKeyPoint getKeyPoints(Mat input)
	{
		colorProcessImage(input);
		
		FeatureDetector blobDetector = getBlobDetector();
		MatOfKeyPoint keypoints = new MatOfKeyPoint();
		blobDetector.detect(input, keypoints);
		
		return keypoints;
	}
	
	public static Mat detectBlobs(Mat input)
	{
		MatOfKeyPoint keypoints = getKeyPoints(input);
		Mat output = new Mat();
		
		System.out.println(keypoints.size());
		
		org.opencv.core.Scalar cores = new org.opencv.core.Scalar(0,0,255);
		Features2d.drawKeypoints(input, keypoints, output, cores, 4);
		
		return output;
	}
	
	private static void colorProcessImage(Mat input)
	{
		double[] target_color = {1.0, 1.0, 0.0};
	    double color_range_min = 0.0;
	    double color_range_max = 255.0;
	    
	    double target_r = target_color[0] * (color_range_max - color_range_min) + color_range_min;
	    double target_g = target_color[1] * (color_range_max - color_range_min) + color_range_min;
	    double target_b = target_color[2] * (color_range_max - color_range_min) + color_range_min;
		
		for(int iRow=0;iRow < input.height(); iRow++)
		{
			for(int iCol=0;iCol < input.width(); iCol++)
			{
				double[] values = input.get(iRow, iCol);
				
				double r = values[0];
			    double g = values[1];
			    double b = values[2];

			    double dr = Math.abs(target_r - r);
			    double dg = Math.abs(target_g - g);
			    double db = Math.abs(target_b - b);

			    double f = 0.2989 * dr + 0.5870 * dg + 0.1140 * db;
			    
			    input.put(iRow, iCol, new byte[]{(byte)f, (byte)f, (byte)f});
			}
		}
	}
	
	private static FeatureDetector getBlobDetector()
	{
		FeatureDetector blobDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
		blobDetector.read("blob_parameters.xml"); // -- ask badal to provide this file
		return blobDetector;
	}
}
