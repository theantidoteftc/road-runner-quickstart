package org.firstinspires.ftc.teamcode.drive.opmode

import android.util.Log
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@Autonomous(name="IMU Reporter")
class IMUReporter : LinearOpMode() {

    override fun runOpMode() {
        // TODO: adjust the names of the following hardware devices to match your configuration
        val imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

        telemetry.addLine("Press play to start reporting IMU")
        telemetry.update()

        waitForStart()

        if(isStopRequested) return

        telemetry.clearAll()
        telemetry.addLine("Running...")
        telemetry.update()

        while (!isStopRequested) {
            val imuHeading = imu.getAngularOrientation().firstAngle
            Log.i("IMUReport", imuHeading.toString())
        }
    }

}