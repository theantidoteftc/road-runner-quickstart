package org.firstinspires.ftc.teamcode.drive.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class pulledoutofthinheir extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(90)));

        Trajectory Stone1 = drive.trajectoryBuilder(new Pose2d(-36.0, -61.0, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-24.0,-36.0, Math.toRadians(180)))
                .build();

        Trajectory Foundation1 = drive.trajectoryBuilder(Stone1.end(), true)
                .lineTo(new Vector2d(48, -36))
                .build();

        Trajectory Stone2 = drive.trajectoryBuilder(Foundation1.end())
                .lineTo(new Vector2d(-32, -36.5))
                .build();

        Trajectory Foundation2 = drive.trajectoryBuilder(Stone2.end(), true)
                .lineTo(new Vector2d(48, -36))
                .build();

        Trajectory Stone3 = drive.trajectoryBuilder(Foundation2.end())
                .lineTo(new Vector2d(-40, -37))
                .build();

        Trajectory Foundation3 = drive.trajectoryBuilder(Stone3.end(), true)
                .lineTo(new Vector2d(48, -37))
                .build();

        Trajectory Stone4 = drive.trajectoryBuilder(Foundation3.end())
                .lineTo(new Vector2d(-48, -37.5))
                .build();

        Trajectory Foundation4 = drive.trajectoryBuilder(Stone4.end(), true)
                .lineTo(new Vector2d(48, -37.5))
                .build();

        Trajectory Park = drive.trajectoryBuilder(Foundation4.end(), true)
                .lineTo(new Vector2d(0, -37.5))
                .build();

        /*Trajectory toFoundation = drive.trajectoryBuilder(toQuarry.end(), true)
                .lineTo(new Vector2d(12, -36))
                .splineTo(new Vector2d(48, -36), Math.toRadians(90))
                .lineTo(new Vector2d(48, -32))
                .build();*/

        drive.followTrajectory(Stone1);
        drive.followTrajectory(Foundation1);
        drive.followTrajectory(Stone2);
        drive.followTrajectory(Foundation2);
        drive.followTrajectory(Stone3);
        drive.followTrajectory(Foundation3);
        drive.followTrajectory(Stone4);
        drive.followTrajectory(Foundation4);
        drive.followTrajectory(Park);
    }
}
