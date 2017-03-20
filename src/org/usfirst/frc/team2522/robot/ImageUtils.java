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
    		camera.setFPS(15);
	    	
    		if (!camera.isConnected())
	    	{
	        	System.out.println(name + " Camera Not Connected.");
	        	camera.free();
	        	camera = null;
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

		if ((ImageUtils.camera != null) && ((camera == null) || (ImageUtils.camera.getName() != camera.getName())))
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
	
	public static List<Rect> processFrame(boolean highCamera, double minAspectRatio, double maxAspectRatio, boolean showDashboard, boolean showImageBlobs, boolean showImageTargets, boolean saveImage)
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
					ImageUtils.saveImage(src_image, rectangles);
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
						
						if (highCamera)
						{
							SmartDashboard.putNumber("Target-Dist", getBoilerTargetDistance(rectangles));
							SmartDashboard.putNumber("Target-Error", getBoilerTargetPixelError(rectangles));
						}
						else
						{
							SmartDashboard.putNumber("Target-Dist", getPegTargetDistance(rectangles));
							SmartDashboard.putNumber("Target-Error", getPegTargetPixelError(rectangles));
						}
					}
					
					if (showDashboard)
					{
						ImageUtils.cameraStream.putFrame(src_image);
					}
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
		return getRectangles(src_image, minAspectRatio, maxAspectRation, null);
	}
	
	public static List<Rect> getRectangles(Mat src_image, double minAspectRatio, double maxAspectRation, String imgName)
	{
		ArrayList<Rect> results = new ArrayList<Rect>();
		
		msk_image = getMask(src_image);
		
		if (imgName != null)
		{
			
			Imgcodecs.imwrite("/home/lvuser/" + imgName+"_msk.png", msk_image);
		}
		
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
		double result = Double.NaN;
		
		if (rectangles.size() == 2)
		{
			result = Math.max(rectangles.get(0).height, rectangles.get(1).height) * -0.7869 + 103.7; 
		}
		
		return result;
	}
	
	public static double getPegRotationError(double range)
	{
		return getPegRotationError(range, null);
	}
	
	public static double getPegRotationError(double range, String imgName)
	{
		double result = Double.NaN;
		
		if (ImageUtils.cameraSink != null)
		{
			if (ImageUtils.cameraSink.grabFrame(src_image) == 0)
			{
				ImageUtils.cameraStream.notifyError(ImageUtils.cameraSink.getError());
				System.out.println(ImageUtils.cameraSink.getError());
			}
			else
			{
				if (imgName != null)
				{
					Imgcodecs.imwrite("/home/lvuser/" + imgName+"_src.png", src_image);
				}
				
				List<Rect> rectangles = getRectangles(src_image, 2.0, 3.0, imgName);
				
				if (imgName != null)
				{
					for(int i = 0; i < rectangles.size(); i++)
					{
						Rect r = rectangles.get(i);
						Imgproc.rectangle(src_image, new Point(r.x, r.y), new Point(r.x+r.width, r.y+r.height), new Scalar(255, 0, 0), 3);
					}
					Imgcodecs.imwrite("/home/lvuser/" + imgName+"_pot.png", src_image);
				}
				
				List<List<Rect>> sets = new ArrayList<List<Rect>>();
				
				while(rectangles.size() > 1)
				{
					List<Rect> sameHeightRects = new ArrayList<Rect>();
					List<Rect> remainingRects = new ArrayList<Rect>();
					
					for(int i = 0; i < rectangles.size(); i++)
					{
						if (sameHeightRects.size() == 0)
						{
							sameHeightRects.add(rectangles.get(i));
						}
						else
						{
							if ((Math.abs(sameHeightRects.get(0).y - rectangles.get(i).y) <= 4)
							 && (Math.abs(sameHeightRects.get(0).height - rectangles.get(i).height) <= 4))
							{
								sameHeightRects.add(rectangles.get(i));
							}
							else
							{
								remainingRects.add(rectangles.get(i));
							}
						}
					}
					
					if (sameHeightRects.size() == 2)
					{
						sets.add(sameHeightRects);
					}
					
					rectangles = remainingRects;
				}
				
				rectangles = new ArrayList<Rect>();
				int maxArea = 0;
				for(int i = 0; i < sets.size(); i++)
				{
					Rect r1 = sets.get(i).get(0);
					Rect r2 = sets.get(i).get(1);
					int area = (r1.height * r1.width) + (r2.height * r2.width);
					
					if (rectangles.size() == 0)
					{
						rectangles = sets.get(i);
						maxArea = area;
					}
					else if (area > maxArea)
					{
						rectangles = sets.get(i);
						maxArea = area;
					}
				}
				
				if (rectangles.size() == 2)
				{
					if (imgName != null)
					{
						for(int i = 0; i < rectangles.size(); i++)
						{
							Rect r = rectangles.get(i);
							Imgproc.rectangle(src_image, new Point(r.x, r.y), new Point(r.x+r.width, r.y+r.height), new Scalar(0, 0, 255), 3);
						}
						Imgcodecs.imwrite("/home/lvuser/" + imgName+"_trg.png", src_image);
					}

					double pixelWidth = Math.max(rectangles.get(0).x + rectangles.get(0).width, rectangles.get(1).x + rectangles.get(1).width) - Math.min(rectangles.get(0).x, rectangles.get(1).x);
					double pe = getPegTargetPixelError(rectangles);

					if (pe != Double.NaN)
					{
						double a = range;
						double b = pe * 10.5 / pixelWidth;
						
//						b = b - 1.0; // adjustment for practice bot camera placement.
SmartDashboard.putNumber("A:", a);
SmartDashboard.putNumber("B:", b);
						result = Math.toDegrees(Math.atan(b / a));
					}
				}
			}
		}
		
		return result;
	}
	
	public static double getPegTargetPixelError(List<Rect> rectangles)
	{
		double result = Double.NaN;
		
		if (rectangles.size() == 2)
		{
			int left = Math.min(rectangles.get(0).x, rectangles.get(1).x);
			int right = Math.max(rectangles.get(0).x + rectangles.get(0).width, rectangles.get(1).x + rectangles.get(1).width);
			int center = left + ((right - left) / 2);
			
			result = center - (imageWidth / 2);
		}
		
		return result;
	}
	
	public static double getBoilerTargetDistance(List<Rect> rectangles)
	{
		double result = Double.NaN;
		
		// TODO: calculate ratio size to range. 
		
		return result;
	}

	public static double getBoilerRotationError(double range)
	{
		double result = Double.NaN;
		
		if (ImageUtils.cameraSink != null)
		{
			if (ImageUtils.cameraSink.grabFrame(src_image) == 0)
			{
				ImageUtils.cameraStream.notifyError(ImageUtils.cameraSink.getError());
				System.out.println(ImageUtils.cameraSink.getError());
			}
			else
			{
				List<Rect> rectangles = getRectangles(src_image, 0.15, 0.5);
				
				List<List<Rect>> sets = new ArrayList<List<Rect>>();
				
				while(rectangles.size() > 1)
				{
					List<Rect> sameWidthtRects = new ArrayList<Rect>();
					List<Rect> remainingRects = new ArrayList<Rect>();
					
					for(int i = 0; i < rectangles.size(); i++)
					{
						if (sameWidthtRects.size() == 0)
						{
							sameWidthtRects.add(rectangles.get(i));
						}
						else
						{
							if ((Math.abs(sameWidthtRects.get(0).x - rectangles.get(i).x) <= 5)
							 && (Math.abs(sameWidthtRects.get(0).width - rectangles.get(i).width) <= 5))
							{
								sameWidthtRects.add(rectangles.get(i));
							}
							else
							{
								remainingRects.add(rectangles.get(i));
							}
						}
					}
					
					if (sameWidthtRects.size() == 2)
					{
						sets.add(sameWidthtRects);
					}
					
					rectangles = remainingRects;
				}
				
				rectangles = new ArrayList<Rect>();
				int maxArea = 0;
				for(int i = 0; i < sets.size(); i++)
				{
					Rect r1 = sets.get(i).get(0);
					Rect r2 = sets.get(i).get(1);
					int area = (r1.height * r1.width) + (r2.height * r2.width);
					
					if (rectangles.size() == 0)
					{
						rectangles = sets.get(i);
						maxArea = area;
					}
					else if (area > maxArea)
					{
						rectangles = sets.get(i);
						maxArea = area;
					}
				}
				
				if (rectangles.size() == 2)
				{
					double pixelHeight = Math.max(rectangles.get(0).y + rectangles.get(0).height, rectangles.get(1).y + rectangles.get(1).height) - Math.min(rectangles.get(0).y, rectangles.get(1).y);
					double pe = getPegTargetPixelError(rectangles);

					if (pe != Double.NaN)
					{
						double a = range;
						double b = pe * 10.0 / pixelHeight;
						
						result = Math.toDegrees(Math.atan(b / a));
					}
				}
			}
		}
		
		return result;
	}
	
	public static double getBoilerTargetPixelError(List<Rect> rectangles)
	{
		double result = Double.NaN;

		if (rectangles.size() == 2)
		{
			int left = Math.min(rectangles.get(0).x, rectangles.get(1).x);
			int right = Math.max(rectangles.get(0).x + rectangles.get(0).width, rectangles.get(1).x + rectangles.get(1).width);
			int center = left + ((right - left) / 2);
			
			result = center - (imageWidth / 2);
		}
		
		return result;
	}
	
	public static void saveImage(Mat src, List<Rect> rectangles)
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

			try
			{
				Imgcodecs.imwrite("/home/lvuser/" + f.getName().replaceAll(".txt", "_src.png"), src);
				Imgcodecs.imwrite("/home/lvuser/" + f.getName().replaceAll(".txt", "_msk.png"), ImageUtils.getMask(src));
				Mat tmp = new Mat();
				src.copyTo(tmp);
				for(int index = 0; index < rectangles.size(); index++)
				{
					Rect r = rectangles.get(index);
					Imgproc.rectangle(tmp, new Point(r.x, r.y), new Point(r.x+r.width, r.y+r.height), new Scalar(0, 0, 255), 3);
				}
				Imgcodecs.imwrite("/home/lvuser/" + f.getName().replaceAll(".txt", "_trg.png"), tmp);
			}
			catch(Exception e)
			{
				e.printStackTrace();
			}
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
			ps = null;
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
