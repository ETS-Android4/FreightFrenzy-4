package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.sensors.Camera;
//import org.firstinspires.ftc.teamcode.util.RingDetectorTimeout;

import java.util.List;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;



//***************************************************************************************************************************
@Autonomous(name = "IsaacArrayDemo")

public class IsaacArrayDemo extends LinearOpMode {
  
  private VuforiaCurrentGame vuforiaUltimateGoal;
  private Blinker expansion_Hub_1;
  private Blinker expansion_Hub_2;
  private DistanceSensor frontIR;
  private DistanceSensor rearIR;
  private DistanceSensor leftIR;
  private DistanceSensor rightIR;
  private ColorSensor color_sensor;
  private DcMotor FLDrive;
  private DcMotor FRDrive;
  private DcMotor BLDrive;
  private DcMotor BRDrive;
  private BNO055IMU imu;

  private Camera camera;

//***************************************************************************************************************************
  public class Orientation {
    
    // declare variables
    Quaternion quat;
    float q0;
    float q1;
    float q2;
    float q3;
    double psi;
    
    double theta;
    double phi;
    Orientation angles;
    List<Axis> foo;
    Acceleration gravity;    
    
    //constructor
    public Orientation(){
      
    }    
    //methods
    void update(){
      quat = imu.getQuaternionOrientation();
      q0 = quat.w;
      q1 = quat.x;
      q2 = quat.y;
      q3 = quat.z;
      //
      // convert quaternion to Euler angles.
      // psi = yaw (or heading)
      psi = Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) / Math.PI * 180;
      // theta = pitch
      theta = Math.asin(2 * (q0 * q2 - q3 * q1)) / Math.PI * 180;
      // phi = roll
      phi = Math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) / Math.PI * 180;
      // store gravity vector from IMU
      gravity = imu.getGravity();
    }
    double getPsi(){
      update();
      return psi; //yaw (heading)
    }
    double getTheta(){
      update();
      return theta; //pitch
    }
    double getPhi(){
      update();
      return phi; //roll
    }
    double getGx(){
      return gravity.xAccel;
    }
    double getGy(){
      return gravity.yAccel;
    }
    double getGz(){
      return gravity.zAccel;
    }
  } //END:   class Orientation
  
//***************************************************************************************************************************  
  public class myPath{
    //declare variables
      double originX = 0;
      double originY = 0;
      double targetX = 0;
      double targetY = 0;
      double initHeading = 0;
      double newHeading;
      double oppHeading;
      String originName,targetName,s;
      double deltaX;
      double deltaY;
      double heading;
      double finalHeading;
      double leastRot;
      double direction;
      double distance;
      double rot_L_fwd,rot_R_fwd,rot_L_rev,rot_R_rev;
    
    //constructor
    public myPath()
    {
      double originX;
      double originY;
      double targetX;
      double targetY;
      double initHeading;
      double newHeading;
      double oppHeading;
      String originName,targetName;
      double deltaX;
      double deltaY;
      double heading;
      double finalHeading;
      double leastRot;
      double direction;
      double distance;
      double rot_L_fwd,rot_R_fwd,rot_L_rev,rot_R_rev;
  
    }
    
    //methods
    double calcMyPath(double originX,double originY,double targetX,double targetY,double initHeading){

      deltaX = targetX - originX;
      deltaY = targetY - originY;
      newHeading = Math.atan2(deltaY,deltaX);
      newHeading = java.lang.Math.toDegrees(newHeading) + 90;
      if(newHeading < 180){
        oppHeading = newHeading + 180;
      }
      else {
        oppHeading = newHeading - 180;
      }
      

      distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
      rot_L_fwd = (360 - initHeading + newHeading) % 360;
      rot_R_fwd = -(360 - newHeading + initHeading) % 360;
      rot_L_rev = (360 - initHeading + oppHeading) % 360;
      rot_R_rev = -(360 - oppHeading + initHeading) % 360;
      leastRot = rot_L_fwd;
      direction = 1;
      if (Math.abs(rot_R_fwd)<Math.abs(leastRot)){
        leastRot = rot_R_fwd;
      }
      if(Math.abs(rot_L_rev)<Math.abs(leastRot)){
        leastRot = rot_L_rev;
        direction = -1;
      }
      if (Math.abs(rot_R_rev)<Math.abs(leastRot)){
        leastRot = rot_R_rev;
        direction = -1;
      }
      finalHeading = initHeading + leastRot;
      return 1;
    }
    double getHeading(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return finalHeading;
    }
    double getDistance(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return distance;
    }
    double getRotation(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return leastRot;
    }
    double getDirection(double originX,double originY,double targetX,double targetY,double initHeading){
      calcMyPath(originX,originY,targetX,targetY,initHeading);
      return direction;
    }
  } // END:  class myPath

//********************************************************************************************************************************
  public class Destination {
      
      //declare variables
      public String destName;
      private float[] Coordinate;
      
      //constructor
      public Destination(String name,float destX,float destY){
        Coordinate = new float[2];
        destName = name;
        Coordinate[0] = destX;
        Coordinate[1] = destY;
      }
      
      //methods
      String getName(){
        return destName;
      }
      float getX(){
        return Coordinate[0];
      }
      float getY(){
        return Coordinate[1];
      }
  } //END:  class Destination

//*************************************************************************************************************************
  public void runOpMode() {    
    
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

    // Initialize camera
    camera = new Camera(hardwareMap, this);
//------------------------------------------------------------------------------------    
    //declare variables for PID control system (proportional-integral-derivative)
    int ringCount = 0;
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
    ElapsedTime msTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//------------------------------------------------------------------------------------
//map hardware and declare variables for IR distance sensors
    
    rightIR = hardwareMap.get(DistanceSensor.class, "right");
    leftIR = hardwareMap.get(DistanceSensor.class, "left");
    frontIR = hardwareMap.get(DistanceSensor.class, "front");
    rearIR = hardwareMap.get(DistanceSensor.class, "rear");
    double leftWall = 0;
    double rightWall = 0;
    double frontWall = 0;
    double backWall = 0;
    double distanceWall = 0;

//------------------------------------------------------------------------------------    
    //create Destination objects
    Destination SHO = new Destination("Shoot",100,-901);
    Destination STS = new Destination("StarterStack",-571,-901);
    Destination TZA = new Destination("TargetZoneA",307,-1501);
    Destination TZB = new Destination("TargetZoneB",904,-904);
    Destination TZC = new Destination("TargetZoneC",1501,-1501);
    Destination TZD = new Destination("TargetZoneDetermined",1502,-1502);
    Destination SLC = new Destination("SL_LeftCtr",-1501,-622);
    Destination SRC = new Destination("SL_RightCtr",-1501,-1238);
    Destination SLT = new Destination("SL_LeftTip",-1212,-627);
    Destination SRT = new Destination("SL_RightTip",-1212,-1238);
    Destination TOW = new Destination("TowerGoal",1790,-888);
    Destination PSL = new Destination("PS_LEFT",1790,-107);
    Destination PSC = new Destination("PS_CENTER",1790,-298);
    Destination PSR = new Destination("PS_RIGHT",1790,-488);  
    Destination LLP = new Destination("LaunchLinePark", 400,-901);
  
//------------------------------------------------------------------------------------  
    // declare variables for motor powers on mecanum drive base
    double FLMotorPwr = 0;
    double FRMotorPwr = 0;
    double BLMotorPwr = 0;
    double BRMotorPwr = 0;
    double maxMtrPwr = 1;
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

//------------------------------------------------------------------------------------    
    //declare variables for VuMark result tracking using VuForia
    VuforiaBase.TrackingResults vuMarkResult;
    float X;
    float Y;
    float Z;
    float Xangle;
    float Yangle;
    float Zangle;
    int autoStage = 0;
    // create Vuforia object
    vuforiaUltimateGoal = new VuforiaCurrentGame();
    // Initialize Vuforia (use default settings).
    vuforiaUltimateGoal.initialize(
        "Ae1X2rH/////AAABmeorpS5ODUuhnNQj0tniyedqUdW5kd2tWpw0i4PSCg4cPvqgm+m+GK+y2xUjIy1fOTLwE3zvW45TabFJ/IbwJuyW7X5dqX04Y4lVpHC82Xs+ZMycEUe43yR7qCD387etZbPK3trbZgJ3ZOG+LbSgpJVxsBVb9s5cDXa9MGzHaxNFbqNxqCDGVMeqADDMzgaYNQRf4QlM/91ss6P45q84o88Eo3S89LevDSVwunBLKlSUoLEwTJh4oNfAzVN5oxggCzA5fdFsgyzZ4Xo8Ud/VEh+RmdXNalAcRfzyqRX/9oE1cLSEEvvQEaZmHjXxGIZ9WrDb84ibNiVmLNHHuCh4/+ZXU6fZ0ceFwfeJwY1UZivO", // vuforiaLicenseKey
        VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
        true, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        true); // useCompetitionFieldTargetLocations


//------------------------------------------------------------------------------------
    // Tensorflow

//    RingDetectorTimeout.Detection rings; // Ring stack
    org.firstinspires.ftc.teamcode.navigation.Destination targetZone = org.firstinspires.ftc.teamcode.navigation.FieldDestinations.TZA; // default Target Zone A

    /*
    rings = RingDetectorTimeout.detect(this, );
    targetZone = rings.targetZone;
    */

//------------------------------------------------------------------------------------    
    //declare & initialize variables for using myPath class
    double startX = 0;
    double startY = 0;
    double endX = 0;
    double endY = 0;
    double initialHeading = 0;
    double currentHeading = 0;
    double finalHeading = 0;
    double initialPitch = 0;
    double targetPitch = 0;
    double targetHeading = 0;
    double targetDistance = 0;
    double targetRotation = 0;
    double errHeading = 0;
    double errDistance = 0;
    double errPitch = 0;
    double targetDirection = 0;
    double angleTol = 0.5;
    double stage = 0;
    int complete = 0;
    String s; //text string for logging

//------------------------------------------------------------------------------------    
    // Prompt user to push start button.
    telemetry.addData("Array2D initialized, ", "Press start to continue...");
    telemetry.update();
    // Wait until user pushes start button.
    waitForStart();
 
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    stage = 0.1;
    msTime.reset();
    
    if (opModeIsActive()) {
      
      // Activate Vuforia software.
      vuforiaUltimateGoal.activate();
      
      
      //set mode for drive base motors  
      FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      
      
      Orientation Pose = new Orientation();
      
      while (opModeIsActive()) {

        if(stage == 0.1){
          s = Double.toString(stage);
          Log.i("***16221_LOG***   calcMyPath start: ",s);

          initialHeading = Pose.getPsi()+90;// +90 points North
          initialPitch = 25; //PLACEHOLDER
          startX = SLC.getX();
          startY = SLC.getY();
          endX = SLT.getX();
          endY = SLT.getY();

          myPath path = new myPath();
          path.calcMyPath(startX,startY,endX,endY,initialHeading);
          targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
          targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
          targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
          targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

          s = Double.toString(stage);
          Log.i("***16221_LOG***   calcMyPath complete: ",s);

          stage += 0.1;
        } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        //rotate
        if(stage == 0.2){
          s = Double.toString(stage);
          Log.i("***16221_LOG***   rotate start: ",s);

          currentHeading = Pose.getPsi()+90; // + 90 points North
          errHeading = targetHeading - currentHeading;

          telemetry.addData("startX: ",startX);
          s = Double.toString(startX);
          Log.i("***16221_LOG***   startX: ",s);
          s = Double.toString(startY);
          Log.i("***16221_LOG***   startY: ",s);
          telemetry.addData("endX: ",endX);
          telemetry.addData("initialHeading: ",initialHeading);
          telemetry.addData("currentHeading: ", currentHeading);
          s = Double.toString(currentHeading);
          Log.i("***16221_LOG***   currentHeading: ",s);
          telemetry.addData("targetHeading: ",targetHeading);
          s = Double.toString(targetHeading);
          Log.i("***16221_LOG***   targetHeading: ",s);
          telemetry.addData("errHeading: ",errHeading);
          s = Double.toString(errHeading);
          Log.i("***16221_LOG***   errHeading: ",s);
          telemetry.update();

          if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
          {
            s = Double.toString(errHeading);
            Log.i("***16221_LOG***   errHeadingCorrection: ",s);
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

          }
          if(errHeading >= -angleTol && errHeading <= angleTol)
          {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage += 0.1;
          }
        } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        //translate - motor mode change & encoder reset
        if (stage == 0.3){
          s = Double.toString(stage);
          Log.i("***16221_LOG***   translate - motor init & reset start: ",s);

          // Reset motor mode change before beginning movement
          FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

          // Reset motor encoder values before beginning movement
          FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

          s = Double.toString(stage);
          Log.i("***16221_LOG***  translate - motor init & reset complete ",s);
          stage += 0.1;
        } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        //translate - move
        if (stage == 0.4){
          s = Double.toString(stage);
          Log.i("***16221_LOG***   Translate - move: ",s);


          complete = move(targetDistance, maxMtrPwr, targetDirection, stage, FLDrive, FRDrive, BLDrive, BRDrive);
          if(complete == 1)
          {
            // stop motors and reset encoders at end
            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            s = Double.toString(stage);
            Log.i("***16221_LOG***   STOP & RESET ENCODERS: ",s);

            s = Double.toString(stage);
            Log.i("***16221_LOG***   Transate - move complete: ",s);
            complete = 0; //reset complete trigger
            stage += 0.1;
          }
        } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // translate - re-align post-move
        if (stage == 0.5){
          s = Double.toString(stage);
          Log.i("***16221_LOG***   re-align post-move begin: ",s);
          //
          s = Double.toString(stage);
          Log.i("***16221_LOG***   re-align post-move complete: ",s);
          stage = 1;
        } // End: translate - re-align post-move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      // calcMyPath
      if(stage==1){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = SLT.getX();
        startY = SLT.getY();
        endX = STS.getX();
        endY = STS.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);
        
        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage==2){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -angleTol && errHeading <= angleTol)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage == 3){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor mode change before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 4){
//         ringCount = camera.getCountOfRings();
         if (ringCount == 0){
           TZD = TZA;
          }
          else if (ringCount == 1){
          TZD = TZB;
          }
          else if (ringCount == 4){
          TZD = TZC;
          }
        stage += 1;
      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 5){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move   


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      // calcMyPath
      if(stage==6){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = SLT.getX();
        startY = SLT.getY();
        endX = SHO.getX();
        endY = SHO.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);

        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage==7){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -angleTol && errHeading <= angleTol)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage ==8){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor mode change before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 9){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Translate - move: ",s);
        
        
        complete = move(targetDistance, maxMtrPwr, targetDirection, stage, FLDrive, FRDrive, BLDrive, BRDrive);
        if(complete == 1)
        {
        // stop motors and reset encoders at end
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   STOP & RESET ENCODERS: ",s); 
        
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Transate - move complete: ",s);
        complete = 0; //reset complete trigger
        stage += 1;
        }
      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 10){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      // calcMyPath
      if(stage==11){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = TZD.getX();
        startY = TZD.getY();
        endX = SHO.getX();
        endY = SHO.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);
        
        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage==12){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -angleTol && errHeading <= angleTol)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage == 13){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor mode change before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 14){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Translate - move: ",s);
        
        
        complete = move(targetDistance, maxMtrPwr, targetDirection, stage, FLDrive, FRDrive, BLDrive, BRDrive);
        if(complete == 1)
        {
        // stop motors and reset encoders at end
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   STOP & RESET ENCODERS: ",s); 
        
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Transate - move complete: ",s);
        complete = 0; //reset complete trigger
        stage += 1;
        }
      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 15){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      // calcMyPath
      if(stage== 16){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = SHO.getX();
        startY = SHO.getY();
        endX = TOW.getX();
        endY = TOW.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);
        
        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage == 17){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -angleTol && errHeading <= angleTol)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage == 18){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor mode change before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 19){
        //shoot
        stage += 1;

      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 20){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      // calcMyPath
      if(stage == 21){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = SHO.getX();
        startY = SHO.getY();
        endX = TZD.getX();
        endY = TZD.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);
        
        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage == 22){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -angleTol && errHeading <= angleTol)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage == 23){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor mode change before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 24){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Translate - move: ",s);
        
        
        complete = move(targetDistance, maxMtrPwr, targetDirection, stage, FLDrive, FRDrive, BLDrive, BRDrive);
        if(complete == 1)
        {
        // stop motors and reset encoders at end
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   STOP & RESET ENCODERS: ",s); 
        
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Transate - move complete: ",s);
        complete = 0; //reset complete trigger
        stage += 1;
        }
      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 25){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // calcMyPath
      if(stage== 26){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = TZD.getX();
        startY = TZD.getY();
        endX = SRT.getX();
        endY = SRT.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);
        
        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage == 27){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -angleTol && errHeading <= angleTol)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage == 28){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor mode change before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 29){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Translate - move: ",s);
        
        
        complete = move(targetDistance, maxMtrPwr, targetDirection, stage, FLDrive, FRDrive, BLDrive, BRDrive);
        if(complete == 1)
        {
        // stop motors and reset encoders at end
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   STOP & RESET ENCODERS: ",s); 
        
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Transate - move complete: ",s);
        complete = 0; //reset complete trigger
        stage += 1;
        }
      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 30){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      // calcMyPath
      if(stage== 31){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath start: ",s);  

        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = SRT.getX();
        startY = SRT.getY();
        endX = TZD.getX();
        endY = TZD.getY();
      
        myPath path = new myPath();
        path.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = path.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =path.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = path.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = path.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting

        s = Double.toString(stage);      
        Log.i("***16221_LOG***   calcMyPath complete: ",s);
        
        stage += 1;
      } // End:  calcMyPath

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //rotate
      if(stage == 32){
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   rotate start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;
        
        telemetry.addData("startX: ",startX);
        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        telemetry.addData("endX: ",endX);
        telemetry.addData("initialHeading: ",initialHeading);
        telemetry.addData("currentHeading: ", currentHeading);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        telemetry.addData("targetHeading: ",targetHeading);
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        telemetry.addData("errHeading: ",errHeading);
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();
        
        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
            {
            s = Double.toString(errHeading);      
            Log.i("***16221_LOG***   errHeadingCorrection: ",s); 
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);

            }
        if(errHeading >= -angleTol && errHeading <= angleTol)
            {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);    
            s = Double.toString(stage);
            Log.i("***16221_LOG***   rotate complete: ",s);
            stage +=1;
            }
      } // End:  rotate

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - motor mode change & encoder reset
      if (stage == 33){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   translate - motor init & reset start: ",s);
        
        // Reset motor mode change before beginning movement
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Reset motor encoder values before beginning movement
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s = Double.toString(stage); 
        Log.i("***16221_LOG***  translate - motor init & reset complete ",s); 
        stage += 1;
      } // End:  translate - motor encoder reset

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
      //translate - move    
      if (stage == 34){
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Translate - move: ",s);
        
        
        complete = move(targetDistance, maxMtrPwr, targetDirection, stage, FLDrive, FRDrive, BLDrive, BRDrive);
        if(complete == 1)
        {
        // stop motors and reset encoders at end
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   STOP & RESET ENCODERS: ",s); 
        
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Transate - move complete: ",s);
        complete = 0; //reset complete trigger
        stage += 1;
        }
      } // End:  translate - move

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // translate - re-align post-move
      if (stage == 35){
        s = Double.toString(stage);         
        Log.i("***16221_LOG***   re-align post-move begin: ",s);
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   re-align post-move complete: ",s);
        stage += 1;
      } // End: translate - re-align post-move
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      // aim to shoot
      if(stage == 36){
        s=Double.toString(stage); 
        Log.i("***16221_LOG***   AIM stage start: ",s);
        
        initialHeading = Pose.getPsi()+90;// +90 points North
        initialPitch = 25; //PLACEHOLDER
        startX = TZD.getX();
        startY = TZD.getY();
        endX = LLP.getX();
        endY = LLP.getY();
      
        myPath aim = new myPath();
        aim.calcMyPath(startX,startY,endX,endY,initialHeading);
        targetHeading = aim.getHeading(startX,startY,endX,endY,initialHeading);
        targetDistance =aim.getDistance(startX,startY,endX,endY,initialHeading);
        targetRotation = aim.getRotation(startX,startY,endX,endY,initialHeading);
        targetDirection = aim.getDirection(startX,startY,endX,endY,initialHeading);//NOTE: if -1 then rotate 180 degrees before shooting
 //       if(targetDirection == -1){
//          targetHeading = targetHeading - 180;
//        }
        targetPitch = 33; //PLACEHOLDER VALUE
        s = Double.toString(stage);      
        Log.i("***16221_LOG***   Aim to shoot complete: ",s);
        stage += 1;
      } //END: aim to shoot

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
      // rotate to shoot
      if(stage == 37){
        s=Double.toString(stage); 
        Log.i("***16221_LOG***   Rotate to shoot stage start: ",s);
        
        currentHeading = Pose.getPsi()+90; // + 90 points North
        errHeading = targetHeading - currentHeading;

        s = Double.toString(startX);      
        Log.i("***16221_LOG***   startX: ",s);    
        s = Double.toString(startY);      
        Log.i("***16221_LOG***   startY: ",s);
        s = Double.toString(currentHeading);      
        Log.i("***16221_LOG***   currentHeading: ",s);  
        s = Double.toString(targetHeading);      
        Log.i("***16221_LOG***   targetHeading: ",s);    
        s = Double.toString(errHeading);      
        Log.i("***16221_LOG***   errHeading: ",s);     
        telemetry.update();

        if (errHeading <= -angleTol || errHeading >= angleTol) //if not aligned straight at target, errHeading=0, then realign
          {
            AngleChange(angleTol, errHeading, FLDrive, FRDrive, BLDrive, BRDrive);
          }
        if(errHeading >= -angleTol && errHeading <= angleTol)
          {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);  
            s = Double.toString(stage);
            Log.i("***16221_LOG***   Rotate to shoot complete: ",s);
            stage += 1;
          } 
        }  //END:  rotate to shoot
          
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      //pitch to shoot
      if(stage == 38){
        s=Double.toString(stage); 
        Log.i("***16221_LOG***   Pitch stage start: ",s);
        
        errPitch = targetPitch - initialPitch;
        //
        s = Double.toString(stage);
        Log.i("***16221_LOG***   Pitch to shoot complete: ",s);
        stage += 1;
      } //END:  pitch to shoot

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
      //shoot
      if(stage == 39){
          s=Double.toString(stage); 
          Log.i("***16221_LOG***  Shoot stage start: ",s);
      
          //
          s = Double.toString(stage);
          Log.i("***16221_LOG***   Shoot complete: ",s);
          stage += 1;
      } //END: shoot
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    } //END:   While(OpModeIsActive)
  } //END:  If(OpModeIsActive)
} //END:  RunOpMode

//********************************************************************************************************************
  public void AngleCorrection(double angleTol, double psi, DcMotor FLDrive, DcMotor FRDrive, DcMotor BLDrive, DcMotor BRDrive)
  {
 
      FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      FLDrive.setDirection(DcMotor.Direction.REVERSE);
      FRDrive.setDirection(DcMotor.Direction.FORWARD);
      BLDrive.setDirection(DcMotor.Direction.REVERSE);
      BRDrive.setDirection(DcMotor.Direction.FORWARD);

//    FLDrive.setPower(0);
//    FRDrive.setPower(0);
//    BLDrive.setPower(0);
//    BRDrive.setPower(0);
    if (psi >= -angleTol && psi <= angleTol)
    {
      FLDrive.setPower(0);
      FRDrive.setPower(0);
      BLDrive.setPower(0);
      BRDrive.setPower(0);
    }
    else if (psi > angleTol) //rotate clockwise
    {
      FLDrive.setPower(0.15);
      FRDrive.setPower(-0.15);
      BLDrive.setPower(0.15);
      BRDrive.setPower(-0.15);
    }
    else if (psi < -angleTol) //rotate counterclockwise
    {
      FLDrive.setPower(-0.15);
      FRDrive.setPower(0.15);      
      BLDrive.setPower(-0.15);
      BRDrive.setPower(0.15);
    }
  } //END:  AngleCorrection

//********************************************************************************************************************  
  public void AngleChange(double angleTol, double psi, DcMotor FLDrive, DcMotor FRDrive, DcMotor BLDrive, DcMotor BRDrive)
  {   

    FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    FLDrive.setDirection(DcMotor.Direction.REVERSE);
    FRDrive.setDirection(DcMotor.Direction.FORWARD);
    BLDrive.setDirection(DcMotor.Direction.REVERSE);
    BRDrive.setDirection(DcMotor.Direction.FORWARD);
    
    if (psi >= -angleTol && psi <= angleTol)
    {
      FLDrive.setPower(0);
      FRDrive.setPower(0);
      BLDrive.setPower(0);
      BRDrive.setPower(0);
    }
    else if (psi > angleTol) //rotate counterclockwise
    {
      FLDrive.setPower(-0.2);
      FRDrive.setPower(0.2);
      BLDrive.setPower(-0.2);
      BRDrive.setPower(0.2);
    }
    else if (psi < -angleTol) //rotate clockwise
    {
      FLDrive.setPower(0.2);
      FRDrive.setPower(-0.2);
      BLDrive.setPower(0.2);
      BRDrive.setPower(-0.2);
    }

  } //END:  AngleChange
  
//********************************************************************************************************************  
  public int move(double distance, double speed, double direction, double stage, DcMotor FLDrive, DcMotor FRDrive, DcMotor BLDrive, DcMotor BRDrive){
      double slip;
      double ticksPerMM = 2.3663; // NOTE:  determined this number emperically by running a set # of ticks and dividing by the actual distance traveled
      int targetTicks = (int) Math.round(ticksPerMM * distance);
      int tolerance = 50; // 10 ticks = 4.23 mm traveled (= 0.166 inches)
      String s;
      int complete = 0;
      int FLcomplete = 0;
      int FRcomplete = 0;
      int BLcomplete = 0;
      int BRcomplete = 0;
  
      // set target position
      FLDrive.setTargetPosition(targetTicks);
      FRDrive.setTargetPosition(targetTicks);
      BLDrive.setTargetPosition(targetTicks);
      BRDrive.setTargetPosition(targetTicks);
      s = Double.toString(targetTicks); 
      Log.i("***16221_LOG***   targetTicks set: ",s);         
        
      // set position tolerannce
      ((DcMotorEx) FLDrive).setTargetPositionTolerance(tolerance);
      ((DcMotorEx) FRDrive).setTargetPositionTolerance(tolerance);
      ((DcMotorEx) BLDrive).setTargetPositionTolerance(tolerance);
      ((DcMotorEx) BRDrive).setTargetPositionTolerance(tolerance);
      s = Double.toString(tolerance); 
      Log.i("***16221_LOG***   tolerance set: ",s); 
      
      if (direction == -1){
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
      } 
      else if (direction == 1){
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
      }
      s = Double.toString(direction); 
      Log.i("***16221_LOG***   direction set: ",s); 


      // turn off each motor as it reaches targetTicks + tolerance, to prevent it 
      // from being pushed beyond tolerance range by other motors that may still
      // active and thus being turned on again during next pass thru the sending loop
      if (FLDrive.getCurrentPosition() >= (targetTicks - tolerance)){
        FLDrive.setPower(0);
        FLcomplete = 1;
      } else {
        FLDrive.setPower(speed);
      }
      if (FRDrive.getCurrentPosition() >= (targetTicks - tolerance)) {
        FRDrive.setPower(0); 
        FRcomplete = 1;
      } else {
        FRDrive.setPower(speed);
      }
      if (BLDrive.getCurrentPosition() >= (targetTicks - tolerance)){
        BLDrive.setPower(0);
        BLcomplete = 1;
      } else {
        BLDrive.setPower(speed);
      }
      if (BRDrive.getCurrentPosition() >= (targetTicks - tolerance)) {
        BRDrive.setPower(0); 
        BRcomplete = 1;
      } else {
        BRDrive.setPower(speed);
      }
      
      s = Double.toString(speed); 
      Log.i("***16221_LOG***   speed set: ",s); 

      // run motors to set position at set speed
      FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
      s = Double.toString(stage);      
      Log.i("***16221_LOG***   current stage: ",s);
      s = Double.toString(FLDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder FL: ",s);
      s = Double.toString(FRDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder FR: ",s); 
      s = Double.toString(BLDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder BL: ",s); 
      s = Double.toString(BRDrive.getCurrentPosition());      
      Log.i("***16221_LOG***   encoder BR: ",s); 
 
      if ((FLcomplete == 1) && (FRcomplete == 1) && (BLcomplete == 1) && (BRcomplete == 1))
      {
        complete = 1;  
      }
      return complete;
  }  // END:  move
//********************************************************************************************************************

}
