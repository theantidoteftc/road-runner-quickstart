package org.firstinspires.ftc.teamcode.drive.ComputerVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class SkyStonePipeLine extends OpenCvPipeline {
    private Mat matYCrCb = new Mat();
    private Mat matCb_left = new Mat();
    private Mat matCb_Right = new Mat();
    private Mat left_block = new Mat();
    private Mat right_block = new Mat();

    public double left;
    public double right;

    public int pattern=2;


    public static int rightStone_leftCoord_X = 205;
    public static int rightStone_leftCoord_Y = 75;
    public static int rightStone_rightCoord_X = 230;
    public static int rightStone_rightCoord_Y = 90;

    public static int leftStone_leftCoord_X = 140;
    public static int leftStone_leftCoord_Y = 75;
    public static int leftStone_rightCoord_X = 160;
    public static int leftStone_rightCoord_Y = 90;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
        int[] left_rect = {
                leftStone_leftCoord_X,
                leftStone_leftCoord_Y,
                leftStone_rightCoord_X ,
                leftStone_rightCoord_Y
        };

        int[] right_rect = {
                rightStone_leftCoord_X,
                rightStone_leftCoord_Y,
                rightStone_rightCoord_X,
                rightStone_rightCoord_Y
        };
        Imgproc.rectangle(
                input,
                new Point(
                        left_rect[0],
                        left_rect[1]),

                new Point(
                        left_rect[2],
                        left_rect[3]),
                new Scalar(0, 255, 0), 1);

        Imgproc.rectangle(
                input,
                new Point(
                        right_rect[0],
                        right_rect[1]),

                new Point(
                        right_rect[2],
                        right_rect[3]),
                new Scalar(0, 0, 255), 1);

        //TODO check if checking if it is null makes it draw on the same frame or the updated one
        left_block = matYCrCb.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
        right_block = matYCrCb.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);


        Core.extractChannel(left_block, matCb_left, 2);
        Core.extractChannel(right_block, matCb_Right, 2);

        Scalar left_mean = Core.mean(matCb_left);
        Scalar right_mean = Core.mean(matCb_Right);
        left=left_mean.val[0];
        right=right_mean.val[0];



        if(Math.abs(left-right)<=15){
            pattern=3;
        }else if(left<right){
            pattern=1;
        }else if(right<left){
            pattern=2;
        }

        return input;

    }
}