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
                .lineToSplineHeading(new Pose2d(-24.0,-33.0, Math.toRadians(180)))
                .build();

        Trajectory Foundation1 = drive.trajectoryBuilder(Stone1.end(), true)
                .splineToConstantHeading(new Vector2d(-8, -37),0)
                .splineTo(new Vector2d(48, -33), 0)
                .build();

        Trajectory Stone2 = drive.trajectoryBuilder(Foundation1.end())
                .splineTo(new Vector2d(-8, -37), Math.toRadians(180))
                .splineTo(new Vector2d(-48, -33), Math.toRadians(180))
                .build();

        Trajectory Foundation2 = drive.trajectoryBuilder(Stone2.end(), true)
                .splineTo(new Vector2d(-8, -37), Math.toRadians(0))
                .splineTo(new Vector2d(48, -33), Math.toRadians(0))
                .build();

        Trajectory Stone3 = drive.trajectoryBuilder(Foundation2.end())
                .splineTo(new Vector2d(-8, -37), Math.toRadians(180))
                .splineTo(new Vector2d(-26.5, -28), Math.toRadians(135))
                .build();

        Trajectory Foundation3 = drive.trajectoryBuilder(Stone3.end(), true)
                .splineTo(new Vector2d(0, -37), Math.toRadians(0))
                .splineTo(new Vector2d(48, -33), Math.toRadians(90))
                .build();

        Trajectory Stone4FoundationPull = drive.trajectoryBuilder(Foundation3.end())
                .splineTo(new Vector2d(24, -48), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -38), Math.toRadians(180))
                .splineTo(new Vector2d(-34.5, -28), Math.toRadians(135))
                .build();

        Trajectory Foundation4 = drive.trajectoryBuilder(Stone4FoundationPull.end(), true)
                .splineTo(new Vector2d(18, -37), Math.toRadians(0))
                .build();

        Trajectory Stone5 = drive.trajectoryBuilder(Foundation4.end(), false)
                .splineTo(new Vector2d(-50.5, -28), Math.toRadians(135))
                .build();

        Trajectory Foundation5 = drive.trajectoryBuilder(Stone5.end(), true)
                .splineTo(new Vector2d(42, -37), Math.toRadians(0))
                .build();

        Trajectory Park = drive.trajectoryBuilder(Foundation5.end(), false)
                .splineToConstantHeading(new Vector2d(0, -34), Math.toRadians(180))
                .build();

        Trajectory returnToHome = drive.trajectoryBuilder(Foundation3.end(), false)
                .lineToSplineHeading(new Pose2d(-36,-60, Math.toRadians(90)))
                .build();

        drive.followTrajectory(Stone1);
        drive.followTrajectory(Foundation1);
        drive.followTrajectory(Stone2);
        drive.followTrajectory(Foundation2);
        drive.followTrajectory(Stone3);
        drive.followTrajectory(Foundation3);
        drive.followTrajectory(Stone4FoundationPull);
        drive.followTrajectory(Foundation4);
        drive.followTrajectory(Stone5);
        drive.followTrajectory(Foundation5);
        drive.followTrajectory(Park);
        //drive.followTrajectory(returnToHome);
    }
}
