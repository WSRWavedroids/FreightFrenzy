/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

        import android.os.Environment;

        import org.opencv.core.Core;
        import org.opencv.core.Mat;
        import org.opencv.core.Point;
        import org.opencv.core.Rect;
        import org.opencv.core.Scalar;
        import org.opencv.imgcodecs.Imgcodecs;
        import org.opencv.imgproc.Imgproc;
        import org.openftc.easyopencv.OpenCvPipeline;

        import java.io.File;

public class DetectionCalculation extends OpenCvPipeline {
    /*
     * An enum to define the randomization
     */
    public enum CapstonePosition
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    /*
     * Some color constants
     */

    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar BLACK = new Scalar(225, 225, 225);

    static final Point REGION1 = new Point(350, 800);
    static final Point REGION2 = new Point(350, 370);

    static final int REGION1_WIDTH = 125;
    static final int REGION1_HEIGHT = 125;
    static final int REGION2_WIDTH = 125;
    static final int REGION2_HEIGHT = 125;

    final int POSITION_THRESHOLD = 140;

    Point region1_pointA = new Point(
            REGION1.x,
            REGION1.y);
    Point region1_pointB = new Point(
            REGION1.x + REGION1_WIDTH,
            REGION1.y + REGION1_HEIGHT);
    Point region2_pointA = new Point(
            REGION2.x,
            REGION2.y);
    Point region2_pointB = new Point(
            REGION2.x + REGION2_WIDTH,
            REGION2.y + REGION2_HEIGHT);

    /*
     * Working variables
     */

    Mat region1_Cb;
    Mat region2_Cb;
    Mat region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;
    int avg2;

    // Volatile since accessed by OpMode thread w/o synchronization
    public volatile CapstonePosition position = getCapstonePosition();

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
    }

    Mat frame = null;

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];


        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        position = getCapstonePosition();

        Scalar color = BLACK;
        if (position == CapstonePosition.LEFT) {
            color = BLUE; // Blue
        } else if (position == CapstonePosition.MIDDLE) {
            color = GREEN; // Green
        } else if (position == CapstonePosition.RIGHT) {
            color = RED; // Red
        }

        /*
         * Left Position:  Blue
         * Middle Position:   Green
         * Right Position:  Red
         * Error: Black/White
         */

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                color, // The color the rectangle is drawn in
                // -1); // Negative thickness means solid fill
                4); // Negative thickness means solid fill
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                color, // The color the rectangle is drawn in
                // -1); // Negative thickness means solid fill
                4); // Negative thickness means solid fill

        frame = input;
        return input;
    }

    public int getAnalysis() {
        return avg1;
    }

    public CapstonePosition getCapstonePosition() {
        if (avg1 > POSITION_THRESHOLD) {
            return CapstonePosition.LEFT;
        } else if (avg2 > POSITION_THRESHOLD) {
            return CapstonePosition.MIDDLE;
        } else {
            return CapstonePosition.RIGHT;
        }
    }

    public void saveLastFrame() {
        File path = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DCIM);
        Imgcodecs.imwrite(path + "/1.jpg", frame);
    }
}
