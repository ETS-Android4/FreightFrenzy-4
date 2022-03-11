package org.firstinspires.ftc.teamcode; //set your package

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TeleDrive;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

@TeleOp(name = "Simple2_RD", group = "TeleDrive")
@Disabled
public class Simple2_RD extends TeleDrive {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime msTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private ColorSensor color_sensor;
    private DistanceSensor frontIR;
    private DistanceSensor rearIR;
    private DistanceSensor leftIR;
    private DistanceSensor rightIR;
    private BNO055IMU imu;

    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;

    double r;
    double robotAngle;
    double rightX;
    double v1;
    double v2;
    double v3;
    double v4;

    @Override
    public void init() {
        super.init();

        //declare variables for quaternion math using BNO055 IMU
        BNO055IMU.Parameters imuParameters;

//------------------------------------------------------------------------------------
        //map to IMU hardware
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        //  set mode to "IMU". This will use the
        // device's gyro and its accelerometer
        // to calculate the relative orientation of the
        // hub and therefore the robot.
        // Gyro gets calibrated automatically in this mode.
        imuParameters.mode = BNO055IMU.SensorMode.NDOF;
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Set acceleration integration algorithm to
        // "naive" which will give velocity & position
        imuParameters.accelerationIntegrationAlgorithm = null;
        // Enable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Initialize variables.
        // Report the initialization to the Driver Station
        telemetry.addData("IMU status", "IMU Initalized.  Calibration started....");

//------------------------------------------------------------------------------------
        //declare variables for PID control system (proportional-integral-derivative)
        double errX = 0;
        double errY = 0;
        double errZ = 0;
        double errX_prev = 0;
        double errY_prev = 0;
        double errZ_prev = 0;
        double dvErrX = 0;
        double dvErrY = 0;
        double dvErrZ = 0;
        double iErrX = 0;
        double iErrY = 0;
        double iErrZ = 0;
        double IRerrX = 0;   // used for IR distance sensor, X direction
        double IRerrY = 0;   // used for IR distance sensor, Y direction
        //********************PID COEFFICIENTS***********************
        double Kp = 0.000833;
        double Kd = -0.0006244;
        double Ki = -0.0000888;
        //***********************************************************

//------------------------------------------------------------------------------------
        //declare variables for timers
        double msTime_now = 0;
        double msTime_prev = 0;
        double dTime = 0;
        // create timer object


//------------------------------------------------------------------------------------
//map hardware and declare variables for IR distance sensors

        rightIR = hardwareMap.get(DistanceSensor.class, "right");
        leftIR = hardwareMap.get(DistanceSensor.class, "left");
        frontIR = hardwareMap.get(DistanceSensor.class, "front");
        rearIR = hardwareMap.get(DistanceSensor.class, "right");
        double leftWall = 0;
        double rightWall = 0;
        double frontWall = 0;
        double backWall = 0;
        double distanceWall = 0;

//------------------------------------------------------------------------------------
        // declare variables for motor powers on mecanum drive base
        double FLMotorPwr = 0;
        double FRMotorPwr = 0;
        double BLMotorPwr = 0;
        double BRMotorPwr = 0;
        double maxMtrPwr = 1;
        double r;
        double robotAngle;
        double rightX;
        double v1;
        double v2;
        double v3;
        double v4;
        // map hardware to variables for mecanum drive motors
        FLDrive  = hardwareMap.dcMotor.get("left_front");
        FRDrive  = hardwareMap.dcMotor.get("right_front");
        BLDrive  = hardwareMap.dcMotor.get("left_rear");
        BRDrive  = hardwareMap.dcMotor.get("right_rear");
        // Set all motors to zero power
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
        // set motor directions for mecanum drive base
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        // Set Zero Power Behavior of all four drive motors
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        msTime.reset();
        telemetry.addData("IMU status", "Simple2_RD initialization completed, starting super.init...");
        telemetry.addData("init","completed");
        telemetry.update();
    }

    public void init_loop() {
        super.init_loop();
        //Init Loop code here
        telemetry.addData("init_loop","completed");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        //Start code here
        runtime.reset();
        msTime.reset();
        telemetry.addData("start","completed");
        telemetry.update();
        loop();
    }

    @Override
    public void loop() {
        super.loop();
        r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;
        v1 = r * Math.cos(robotAngle) + rightX;
        v2 = r * Math.sin(robotAngle) - rightX;
        v3 = r * Math.sin(robotAngle) + rightX;
        v4 = r * Math.cos(robotAngle) - rightX;

        telemetry.addData("v1", v1);
        telemetry.addData("v2", v2);
        telemetry.addData("v3", v3);
        telemetry.addData("v4", v4);
        telemetry.update();

        FLDrive.setPower(v1);
        FRDrive.setPower(v2);
        BLDrive.setPower(v3);
        BRDrive.setPower(v4);
    }

    @Override
    public void stop() {
        super.stop();
        //Stop code here
    }
}