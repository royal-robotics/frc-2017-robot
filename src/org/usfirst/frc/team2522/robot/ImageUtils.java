package org.usfirst.frc.team2522.robot;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;

//Provides static members to detect blobs
public class ImageUtils {
	
	public static KeyPoint[] getKeyPoints(Mat input)
	{
		colorProcessImage(input);
		
		FeatureDetector blobDetector = getBlobDetector();
		MatOfKeyPoint keypoints = new MatOfKeyPoint();
		blobDetector.detect(input, keypoints);
		
		return keypoints.toArray();
	}
	
	public static Mat detectBlobs(Mat input)
	{
		colorProcessImage(input);
		
		FeatureDetector blobDetector = getBlobDetector();
		MatOfKeyPoint keypoints = new MatOfKeyPoint();
		blobDetector.detect(input, keypoints);
		
		Mat output = new Mat();
		
		Features2d.drawKeypoints(input, keypoints, output);
		
		return output;
	}
	
	private static void colorProcessImage(Mat input)
	{
		for(int iRow=0;iRow < input.height(); iRow++)
		{
			for(int iCol=0;iCol < input.width(); iCol++)
			{
				//transform color pixels to have brighter values for green and vice versa 
				// -- ask Badal
			}
		}
	}
	
	private static FeatureDetector getBlobDetector()
	{
		FeatureDetector blobDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
		blobDetector.write("blob_parameters.xml"); // -- ask badal to provide this file
		return blobDetector;
	}
}