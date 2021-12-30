package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorPipeline extends OpenCvPipeline {

    //private Point SIDE_BOTTOM_LEFT = new Point(0, 211.2);
    //private Point SIDE_TOP_RIGHT = new Point(114.56,249.6);
    private static final Point REGION1_BOTTOM_LEFT = new Point(205.28,211.2);
    private static final Point REGION1_TOP_RIGHT = new Point(320,249.6);
    private static final Point REGION2_BOTTOM_LEFT = new Point(588.8, 211.2);
    private static final Point REGION2_TOP_RIGHT = new Point(640,249.6);

    private static final Scalar GREEN = new Scalar(0, 255, 0);

    //Mat Side;
    Mat Region1, Region2;
    //Mat Color = new Mat();
    Mat HLS = new Mat();
    //int avgSide;

    private int avg1, avg2;
    private int barcode;
    public static int[] HLS1 = new int[3];
    public static int[] HLS2 = new int[3];

    //public enum Barcode {
    //    ONE, TWO, THREE
    //}
    //private Barcode number = Barcode.THREE;
    //private volatile Barcode number = Barcode.THREE;

    //convert mat into color depending on scale
   /* public void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2HSV_FULL);
        //vCore.extractChannel(YCrCb, Color, 1);
    }*/
/*
    @Override
    public void init(Mat firstFrame) {
        //inputToCb(firstFrame);

        //Side = Color.submat(new Rect(SIDE_BOTTOM_LEFT,SIDE_TOP_RIGHT));
        Region1 = Color.submat(new Rect(REGION1_BOTTOM_LEFT,REGION1_TOP_RIGHT));
        Region2 = Color.submat(new Rect(REGION2_BOTTOM_LEFT,REGION2_TOP_RIGHT));

    }
    */

    @Override
    public Mat processFrame(Mat input) {

        //inputToCb(input);
        Imgproc.cvtColor(input, HLS, Imgproc.COLOR_RGB2HLS);

        Region1 = HLS.submat(new Rect(REGION1_BOTTOM_LEFT,REGION1_TOP_RIGHT));
        Region2 = HLS.submat(new Rect(REGION2_BOTTOM_LEFT,REGION2_TOP_RIGHT));
        //avgSide = (int) Core.mean(Side).val[0];
        avg1 = (int) Core.mean(Region1).val[0];
        avg2 = (int) Core.mean(Region2).val[0];

        for (int i = 0; i < 3; i++){
            // Finds the average HSV value for each channel of interest (The "i" representing the channel of interest)
            HLS1[i] = (int) Core.mean(Region1).val[i];
            HLS2[i] = (int) Core.mean(Region2).val[i];
        }


        //find max color value for both side + barcodes
        //test color space for biggest contrast between
        //grey + Red/blue (side)
        //blue/red + shipping element color (barcode)
        //need a range for the same color incase element is on third barcode (outside of cam view)

            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    SIDE_BOTTOM_LEFT, // First point which defines the rectangle
                    SIDE_TOP_RIGHT, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2 // Thickness of the rectangle lines
            );*/

        Imgproc.rectangle(
                HLS,
                REGION1_BOTTOM_LEFT,
                REGION1_TOP_RIGHT,
                GREEN,
                2
        );

        Imgproc.rectangle(
                HLS,
                REGION2_BOTTOM_LEFT,
                REGION2_TOP_RIGHT,
                GREEN,
                2
        );

        findBarcode();

        return HLS;
    }

    //determine which barcode has the element
    public void findBarcode() {
        if (Math.abs(avg1 - avg2) <= 10) {
            barcode = 3;
        }
        else if (avg2 > avg1) {
            barcode = 2;
        }
        else if (avg1 > avg2) {
            barcode = 1;
        }

    }

    //return barcode constant
    public int getBarcode() {
        return barcode;
    }

    //get methods for average color values
    public int getAvg1() {
        return avg1;
    }

    public int getAvg2() {
        return avg2;
    }

}