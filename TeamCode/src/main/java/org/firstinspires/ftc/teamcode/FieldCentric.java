package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class FieldCentric {
    // Variable setup, all will be explained within code
    private DcMotor[] motors;
    private double[] wheelAngles;
    private double r;
    private double theta;
    private double currentAngle;
    private BNO055IMU imu;

    private double[] wheelPowers;

    public void setUp(DcMotor[] motors, double[] wheelAngles, BNO055IMU imu) throws Exception {
        // Check if we have angles for every motor, and vice versa
        if (motors.length != wheelAngles.length) {
            throw new java.lang.Exception("Motor and wheelAngle arrays do not have same length.\nCheck your code!!!");
        }

        this.motors = motors;
        this.wheelAngles = wheelAngles;
        this.wheelPowers = new double[motors.length];
        this.imu = imu;
    }

    private void getAngle() {
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;
    }

    /**
     * Run every loop to drive robot using field centricity
     * @param x The input to control robot's x movement
     * @param y The input to control robot's y movement
     * @param turn The input to control robot's turn
     */
    public void Drive(double x, double y, double turn) {
        /*
            Get the current angle
         */
        getAngle();

        /*
            Set r (for polar coords) to the distance of the point (x,y) from (0,0)
            (The speed)
         */
        r = Math.sqrt(x*x+y*y);

        /*
            Set theta (for polar coords) to the theta of the point (x,y) to the x-axis at the origin
            (The direction to go)
         */
        theta = Math.atan2(x, y);

        /*
            Add current angle to account for rotation (since we are getting the theta from a controller
            axis (-1.0 to 1.0) we don't know the angle we are currently at
         */
        double newTheta = theta + currentAngle;

        /*
            Get the angle of the wheel and subtract the newTheta (because newTheta is clockwise and
            math is counter-clockwise)
         */
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(Math.sin(wheelAngles[i] - newTheta) * r + turn);
            wheelPowers[i] = motors[i].getPower();
        }
    }
}
