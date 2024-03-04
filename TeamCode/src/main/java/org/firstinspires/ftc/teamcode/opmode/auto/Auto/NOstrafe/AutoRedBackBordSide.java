package org.firstinspires.ftc.teamcode.opmode.auto.Auto.NOstrafe;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.RedBlobDetectionPipeline;
import org.firstinspires.ftc.teamcode.common.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Autonomous(name = "NoStrafeAutoRedBackBordSide")
public class AutoRedBackBordSide extends LinearOpMode {
    boolean starting = true;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpelnCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    public int rect = -1;
    RedBlobDetectionPipeline redBlobDetectionPipeline = new RedBlobDetectionPipeline();

    //---------------

    //---------------
    @Override
    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Rect_Num:", rect);


        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //---------------------------
        TrajectorySequence leftPropStart = drive.trajectorySequenceBuilder(new Pose2d(15, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(15, -45), Math.toRadians(130))
                .build(); // to prop line
        TrajectorySequence rightPropStart = drive.trajectorySequenceBuilder(new Pose2d(15, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(34, -35), Math.toRadians(180))
                .build(); // to prop line
        TrajectorySequence midPropStart = drive.trajectorySequenceBuilder(new Pose2d(15, -60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(15, -40, Math.toRadians(90)))
                .build(); // to prop line

//        -----------
        TrajectorySequence leftPropToBkB = drive.trajectorySequenceBuilder(leftPropStart.end())
                .setReversed(true)
                .splineTo(new Vector2d(38, -30), Math.toRadians(0))
                .setReversed(false)
                .build(); //to back bord
        TrajectorySequence rightPropToBkB = drive.trajectorySequenceBuilder(rightPropStart.end())
                .setReversed(true)
                .splineTo(new Vector2d(38, -41), Math.toRadians(0))
                .setReversed(false)
                .build(); //to back bord
        TrajectorySequence midPropToBkB = drive.trajectorySequenceBuilder(midPropStart.end())
                .setReversed(true)
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .setReversed(false)
                .build(); //to back bord
        // --- take more from stack ---
        TrajectorySequence leftGetMoreFromStack = drive.trajectorySequenceBuilder(leftPropToBkB.end())
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
//                .setReversed(false)
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
                .build();
        TrajectorySequence rightGetMoreFromStack = drive.trajectorySequenceBuilder(rightPropToBkB.end())
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
//                .setReversed(false)
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
                .build();
        TrajectorySequence midGetMoreFromStack = drive.trajectorySequenceBuilder(midPropToBkB.end())
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
//                .setReversed(false)
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
                .splineTo(new Vector2d(20, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -11.5), Math.toRadians(180))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(20, -11.5))
                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                .waitSeconds(1)
                .build();
        // PARKING
        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(38, -35, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(45, -35))
                .build();
/*        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(leftPropToBkB.end())
                .splineToConstantHeading(new Vector2d(50, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(rightPropToBkB.end())
                .splineToConstantHeading(new Vector2d(50, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();
        TrajectorySequence parkMid = drive.trajectorySequenceBuilder(midPropToBkB.end())
                .splineToConstantHeading(new Vector2d(50, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60, -60), Math.toRadians(0))
                .build();*/
        //--------------------------


        waitForStart();

        while (opModeIsActive()) {
            rect = redBlobDetectionPipeline.getPropPose();
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Rect_Num:", rect);
            drive.setPoseEstimate(new Pose2d(15, -60, Math.toRadians(90)));
            telemetry.update();
            if (starting){
                switch (rect){
                    case 0:
                        telemetry.addData("rect", "0");
                        drive.followTrajectorySequence(leftPropStart); // get to line
                        // move arms put (purple pixel on the right line)
                        drive.followTrajectorySequence(leftPropToBkB); // get to back bord
                        // move arms (put yellow pixel on the right place on back bord)
                        drive.followTrajectorySequence(leftGetMoreFromStack);// back and forth to get more from stack
                        drive.followTrajectorySequence(park); //parking
                        break;
                    case 1:
                        telemetry.addData("rect", "1");
                        drive.followTrajectorySequence(midPropStart); // get to line
                        // move arms (put purple pixel on the right line)
                        drive.followTrajectorySequence(midPropToBkB); // get to back bord
                        // move arms (put yellow pixel on the right place on back bord)
                        drive.followTrajectorySequence(midGetMoreFromStack);// back and forth to get more from stack
                        drive.followTrajectorySequence(park); //parking
                        break;
                    case 2:
                        telemetry.addData("rect", "2");
                        drive.followTrajectorySequence(rightPropStart); // get to line
                        // move arms (put purple pixel on the right line)
                        drive.followTrajectorySequence(rightPropToBkB); // get to back bord
                        // move arms (put yellow pixel on the right place on back bord)
                        // move arms (put yellow pixel on the right place on back bord)
                        drive.followTrajectorySequence(rightGetMoreFromStack);// back and forth to get more from stack
                        drive.followTrajectorySequence(park); //parking
                        break;
                    case -1:
                        telemetry.addData("rect", "didn't find prop line");
                        break;
                }
/*                if (rect == 0){
                    telemetry.addData("rect", "0");
                    drive.followTrajectorySequence(leftPropStart); // get to line
                    // move arms put (purple pixel on the right line)
                    drive.followTrajectorySequence(leftPropToBkB); // get to back bord
                    // move arms (put yellow pixel on the right place on back bord)
                    drive.followTrajectorySequence(leftGetMoreFromStack);
                    drive.followTrajectorySequence(park); //parking
                }else if (rect == 1){
                    telemetry.addData("rect", "1");
                    drive.followTrajectorySequence(midPropStart); // get to line
                    // move arms (put purple pixel on the right line)
                    drive.followTrajectorySequence(midPropToBkB); // get to back bord
                    // move arms (put yellow pixel on the right place on back bord)
                    // move arms (put yellow pixel on the right place on back bord)
                    drive.followTrajectorySequence(midGetMoreFromStack);
                    drive.followTrajectorySequence(park); //parking
                    } else if (rect == 2){
                    telemetry.addData("rect", "2");
                    drive.followTrajectorySequence(rightPropStart); // get to line
                    // move arms (put purple pixel on the right line)
                    drive.followTrajectorySequence(rightPropToBkB); // get to back bord
                    // move arms (put yellow pixel on the right place on back bord)
                    // move arms (put yellow pixel on the right place on back bord)
                    drive.followTrajectorySequence(rightGetMoreFromStack);
                    drive.followTrajectorySequence(park); //parking
                }
                else {
                    telemetry.addData("rect", "noRect");
                }*/
                telemetry.update();
                starting = false;
            }


            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }
    private void initOpenCV() {
        telemetry.addData("propPoseWithOtherFile", redBlobDetectionPipeline.getPropPose());
        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(redBlobDetectionPipeline);

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
/*
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
        Scalar rightRectColor = new Scalar(0.0, 255.0, 0.0);
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                Rect leftRect = new Rect(20, 150, 300, 450);
                Rect midRect = new Rect(490, 100, 300, 550);
                Rect rightRect = new Rect(950, 150, 300, 450);

                if (leftRect.contains(new Point(cX + 10, cY + 20))){
                    Imgproc.rectangle(input, leftRect, rightRectColor, 2);
                    Imgproc.rectangle(input, midRect, rectColor, 2);
                    Imgproc.rectangle(input, rightRect, rectColor, 2);
                    telemetry.addData("rect:","left rect");
                    rect = 0;
                    telemetry.addData("Rect_Num:", rect);
                }else if (rightRect.contains(new Point(cX + 10, cY + 20))){
                    Imgproc.rectangle(input, leftRect, rectColor, 2);
                    Imgproc.rectangle(input, midRect, rectColor, 2);
                    Imgproc.rectangle(input, rightRect, rightRectColor, 2);
                    telemetry.addData("rect:","right rect");
                    rect = 2;
                }else if (midRect.contains(new Point(cX + 10, cY + 20))){
                    Imgproc.rectangle(input, leftRect, rectColor, 2);
                    Imgproc.rectangle(input, midRect, rightRectColor, 2);
                    Imgproc.rectangle(input, rightRect, rectColor, 2);
                    telemetry.addData("rect:","mid rect");
                    rect = 1;
                }else {
                    Imgproc.rectangle(input, leftRect, rectColor, 2);
                    Imgproc.rectangle(input, midRect, rectColor, 2);
                    Imgproc.rectangle(input, rightRect, rectColor, 2);
                    telemetry.addData("rect:","NO RECT, last rect was" + rect);
                    rect = -1;
                }



                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);


            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat yellowMask = new Mat();
//            Imgproc.cvtColor(frame, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(110, 100, 100);
            Scalar upperYellow = new Scalar(160, 255, 255);



            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

*/
}
