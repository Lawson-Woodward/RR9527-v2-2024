/*

THIS CODE IS NOT USED IN THE ACTUAL ROBOT!!!!!!

*/



package org.firstinspires.ftc.teamcode.commands;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

    public enum TSEPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point MIDDLE_ANCHOR_POINT = new Point(95, 150); //I think this is the actual one we edit! Increase x goes right, increase y goes down.
    private static Point LEFT_ANCHOR_POINT = new Point(95, 30);
    private static Point RIGHT_ANCHOR_POINT = new Point(95, 270);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 30;

    // Color definitions
    private final Scalar

            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255),
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255),
            GREY = new Scalar(127, 127, 127);



    // Anchor point definitions
    Point pos1_pointA = new Point(
            MIDDLE_ANCHOR_POINT.x,
            MIDDLE_ANCHOR_POINT.y);
    Point pos1_pointB = new Point(
            MIDDLE_ANCHOR_POINT.x + REGION_WIDTH,
            MIDDLE_ANCHOR_POINT.y + REGION_HEIGHT);

    Point pos2_pointA = new Point(
            LEFT_ANCHOR_POINT.x,
            LEFT_ANCHOR_POINT.y);
    Point pos2_pointB = new Point(
            LEFT_ANCHOR_POINT.x + REGION_WIDTH,
            LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point pos3_pointA = new Point(
            RIGHT_ANCHOR_POINT.x,
            RIGHT_ANCHOR_POINT.y);
    Point pos3_pointB = new Point(
            RIGHT_ANCHOR_POINT.x + REGION_WIDTH,
            RIGHT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile TSEPosition position = TSEPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Get the submat frame, and then sum all the values
        Mat middleMat = input.submat(new Rect(pos1_pointA, pos1_pointB));
        Scalar midSumColors = Core.sumElems(middleMat);

        // Get the minimum RGB value from every single channel
        double maxColor = Math.max(midSumColors.val[0], Math.max(midSumColors.val[1], midSumColors.val[2]));

        // Change the bounding box color based on the sleeve color
        if (midSumColors.val[0] == maxColor && midSumColors.val[0] > 100) {
            position = TSEPosition.MIDDLE;
            Imgproc.rectangle(
                    input,
                    pos1_pointA,
                    pos1_pointB,
                    RED,
                    2
            );
        } else if (midSumColors.val[2] == maxColor && midSumColors.val[2] > 100) {
            position = TSEPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    pos1_pointA,
                    pos1_pointB,
                    BLUE,
                    2
            );
        } else {
            position = TSEPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    pos1_pointA,
                    pos1_pointB,
                    GREY,
                    2
            );
        }

        // Release and return input
        middleMat.release();





        // Get the submat frame, and then sum all the values
        Mat leftMat = input.submat(new Rect(pos2_pointA, pos2_pointB));
        Scalar leftSumColors = Core.sumElems(leftMat);

        maxColor = Math.max(leftSumColors.val[0], Math.max(leftSumColors.val[1], leftSumColors.val[2]));

        // Change the bounding box color based on the sleeve color
        if (leftSumColors.val[0] == maxColor && leftSumColors.val[0] > 100) {
            position = TSEPosition.MIDDLE;
            Imgproc.rectangle(
                    input,
                    pos2_pointA,
                    pos2_pointB,
                    RED,
                    2
            );
        } else if (leftSumColors.val[2] == maxColor && leftSumColors.val[2] > 100) {
            position = TSEPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    pos2_pointA,
                    pos2_pointB,
                    BLUE,
                    2
            );
        } else {
            position = TSEPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    pos2_pointA,
                    pos2_pointB,
                    GREY,
                    2
            );
        }

        // Release and return input
        leftMat.release();






        // Get the submat frame, and then sum all the values
        Mat rightMat = input.submat(new Rect(pos3_pointA, pos3_pointB));
        Scalar rightSumColors = Core.sumElems(rightMat);

        maxColor = Math.max(rightSumColors.val[0], Math.max(rightSumColors.val[1], rightSumColors.val[2]));

        // Change the bounding box color based on the sleeve color
        if (rightSumColors.val[0] == maxColor && rightSumColors.val[0] > 100) {
            position = TSEPosition.MIDDLE;
            Imgproc.rectangle(
                    input,
                    pos3_pointA,
                    pos3_pointB,
                    RED,
                    2
            );
        } else if (rightSumColors.val[2] == maxColor && rightSumColors.val[2] > 100) {
            position = TSEPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    pos3_pointA,
                    pos3_pointB,
                    BLUE,
                    2
            );
        } else {
            position = TSEPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    pos3_pointA,
                    pos3_pointB,
                    GREY,
                    2
            );
        }

        // Release and return input
        rightMat.release();





        return input;
    }

    // Returns an enum being the current position where the robot will park
    public TSEPosition getPosition() {
        return position;
    }

    public int[] getColors(Mat input) {
        Mat middleMat = input.submat(new Rect(pos1_pointA, pos1_pointB));
        Scalar midSumColors = Core.sumElems(middleMat);

        // Get the minimum RGB value from every single channel
        double maxColor = Math.max(midSumColors.val[0], Math.max(midSumColors.val[1], midSumColors.val[2]));

        int[] colors = new int[] {};
        return colors;
    }

}