package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetect extends OpenCvPipeline {

   private static final Point REGION2_BOTTOM_LEFT = new Point(343.70 - 100,  108 + 30);
   private static final Point REGION2_TOP_RIGHT = new Point(426.67 - 100,228 + 30);

   private static final Scalar GREEN =  new Scalar(0, 255, 0); //RGB
   private static Value number;
   private static Scalar avg1;

   //Mat Side;
   Mat Region1;
   //Mat Color = new Mat();
   //Mat HLS = new Mat();
   //int avgSide;

   //private int avg1, avg2;
   private static int threshold = 158;
   //private static int barcode;
   public static int[] HLS1 = new int[3];
   public static int[] HLS2 = new int[3];

   public enum Value {
       ONE, TWO, THREE
   }

   @Override
   public Mat processFrame(Mat input) {

      //inputToCb(input);
      //Imgproc.cvtColor(input, HLS, Imgproc.COLOR_RGB2HLS);

      Region1 = input.submat(new Rect(REGION2_BOTTOM_LEFT,REGION2_TOP_RIGHT));

      avg1 = Core.mean(Region1);

      Imgproc.rectangle(
              input,
              REGION2_BOTTOM_LEFT,
              REGION2_TOP_RIGHT,
              GREEN,
              2
      );

      findBarcode();

      return input;
   }

   public static void findBarcode() {
      if (Math.abs(avg1.val[0] - threshold) <= 5) {
         number = Value.ONE;
      }
      else if (avg1.val[0] > threshold) {
         number = Value.TWO;
      }
      else if (avg1.val[0] < threshold) {
         number = Value.THREE;
      }
   }


   //return barcode constant
   public static Value getBarcode() {
      return number;
   }

   //get methods for average color values
   public double getAvg1() {
      return avg1.val[0];
   }

   public int getThreshold() { return threshold; }


}