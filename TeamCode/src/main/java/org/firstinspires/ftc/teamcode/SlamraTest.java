package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="SlamraTest", group="Test")
public class SlamraTest extends LinearOpMode {

    public void runOpMode() {
        // This is the transformation between the center of the camera and the center of the robot
        Transform2d cameraToRobot = new Transform2d();
        // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
        // Set to the starting pose of the robot
        Pose2d startingPose = new Pose2d(1, 1, new Rotation2d());

        T265Camera slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose); // Useful if your robot doesn't start at the field-relative origin

        // Call this when you're ready to get camera updates
        slamra.start();

        // Now we can grab our last received pose in our main thread
        while (opModeIsActive()) {
            slamra.getLastReceivedCameraUpdate();
        }
    }
}
