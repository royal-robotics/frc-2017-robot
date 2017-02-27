package org.usfirst.frc.team2522.robot;

import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgcodecs.Imgcodecs;

public class ImageUtils {

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
