package org.firstinspires.ftc.teamcode.drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;
import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.Subassembly;

import java.util.Arrays;
import java.util.Hashtable;

/**
 * To create a new DriveBase from this pattern, extend this class,
 * define each motor from the hardware map, and
 * Override initMotorConfigurations.
 * Be sure to set the wheelDiameterInches and wheelBaseWidth to suit.
 * You may also wish to adjust the power settings by Overriding initDriveSpeedConfigurations.
 * When using the driveBase in your OpMode, declare a variable of the appropriate DriveBase subclass type
 * in your OpMode class and assign a new instance of that subclass, passing in the hardware map from the OpMode.
 */
@ExportClassToBlocks
public abstract class DriveBase implements Subassembly, HardwareDevice {
    protected HardwareMap hardwareMap;

    /**
     * When instantiating a DriveBase subclass, pass in the hardware map from the OpMode.
     * @param hwMap the hardware map from the opmode
     */
    public DriveBase(HardwareMap hwMap) {
        hardwareMap = hwMap;
        resetDeviceConfigurationForOpMode();
    }
    /**
     * Constants to represent drive speeds that are useful.  Set power values for each in initDriveSpeedConfiguration().
     */
    public enum DriveSpeed {
        STOP,
        ANGLE_CORRECTION,
        SLOW,
        FAST,
        MAX
    }

    // Wheel diameter tells us how far it will travel.
    // Metric
    public double wheelDiameterMM = 96;
    public double wheelRotationDistanceMM = Math.PI * wheelDiameterMM;
    public double motorRotationsPerMM = 1 / wheelRotationDistanceMM;
    // English
    public double wheelDiameterInches = Conversions.mmToInches(wheelDiameterMM);
    public double wheelRotationDistanceInches = Math.PI * wheelDiameterInches;
    public double motorRotationsPerInch = 1 / wheelRotationDistanceInches;

    public double wheelBaseWidth = 16; // inches
    public double wheelBaseLengthMM = 360;
    public double inchwormFrontMM = 216; // front to pivot point
    public double inchwormRearMM = 144; // rear to pivot point

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-223-rpm-3-3-5v-encoder/
    public double motorEncoderEventsPerRotation = 753.2;
    public double motorEncoderEventsPerInch = motorRotationsPerInch * motorEncoderEventsPerRotation;
    public double motorEncoderEventsPerMM = motorRotationsPerMM * motorEncoderEventsPerRotation;

    public double maxPower = 1; // range of 0 to 1

    /**
     * A predefined set of travel directions. Set motor configurations for each direction in initMotorConfigurations().
     */
    public enum TravelDirection {
        base,

        forward,
        reverse,
        pivotLeft,
        pivotRight,

        strafeLeft,
        strafeLeftForward,
        strafeLeftBackward,

        strafeRight,
        strafeRightForward,
        strafeRightBackward,
        pitch
    }


    protected Hashtable<TravelDirection, DcMotorSimple.Direction[]> motorConfigurations = new Hashtable<>();
    protected DcMotor[] wheelMotors = new DcMotor[]{};

    /**
     * Add hashtable entries to Hashtable&lt;TravelDirection, DcMotorSimple.Direction[]&gt; motorConfigurations,
     * representing the directions each motor must turn to send the bot in a given direction.
     * This is mostly useful for autonomous driving; however, teleOp may wish to use these to understand
     * motor directions when handling user input from a gamepad.
     */
    protected void initMotorConfigurations() {
        // define a series of motors for the drivebase here.
        // ex: wheelMotors = new DcMotor[] { leftFront, leftRear, rightRear, rightFront };

        // add hashtable entries to the motorConfigurations here, representing the directions each motor must turn
        // in order for the bot to travel in a given direction.
        // ex: motorConfigurations.put(TravelDirection.forward, Direction.FORWARD);
        //

    }

    /**
     * Returns the defined array of directions for motors, as set in the initMotorConfigurations method.
     * @param travelDirection which direction the bot should travel
     * @return the defined motor directions, or null if no matching configuration has been defined.
     */
    public DcMotorSimple.Direction[] getMotorConfigurations(TravelDirection travelDirection) {
        return motorConfigurations.getOrDefault(travelDirection, null);
    }

    protected Hashtable<DriveSpeed, Double> driveSpeedConfigurations = new Hashtable<>();

    /**
     * Set up the standard power constants for the different drive speeds.
     */
    protected void initDriveSpeedConfigurations()  {
        // add hashtable entries to the driveSpeedConfigurations here, based on the configured motors and wheels.
        driveSpeedConfigurations.put(DriveSpeed.STOP, 0.0);
        driveSpeedConfigurations.put(DriveSpeed.SLOW, 0.2);
        driveSpeedConfigurations.put(DriveSpeed.FAST, 0.8);
        driveSpeedConfigurations.put(DriveSpeed.MAX, 1.0);
    }

    /**
     * Returns the power value for the given drive speed, or null.
     * @param driveSpeed how fast we should go - use the enum
     * @return the current power for the drive speed
     */
    public double getDriveSpeedPower(DriveSpeed driveSpeed) {
        return driveSpeedConfigurations.getOrDefault(driveSpeed, (double) 0);
    }

    /**
     * Configure motors in initMotorConfigurations.
     * @return the series, in order as configured, of motor objects that are relevant to this drivebase.
     */
    public DcMotor[] getMotors() {
        return wheelMotors;
    }

    /**
     * Initialize all motors to run using encoders.
     * Set travel to forward and power to  0.
     */
    @ExportToBlocks(
            heading = "drivebase.initMotors",
            comment = "Initialize all drive motors",
            tooltip = "Stop, reset, brake, set to default directions, set power to 0",
            color = 200)
    public void initMotors() {

        initMotorConfigurations();
        initDriveSpeedConfigurations();

        // Output all configuration information for the drive base.
        RobotLog.i("*******  Drive Base Configuration *******");
        RobotLog.i("Wheel diameter (inches): " + String.valueOf(wheelDiameterInches));
        RobotLog.i("Floor distance for one wheel rotation (inches): " + String.valueOf(wheelRotationDistanceInches));
        RobotLog.i("Rotations per inch of floor distance: " + String.valueOf(motorRotationsPerInch));
        RobotLog.i("Wheel base width (inches): " + String.valueOf(wheelBaseWidth));
        RobotLog.i("Motor encoder events per wheel rotation: " + String.valueOf(motorEncoderEventsPerRotation));
        RobotLog.i("Motor encoder events per inch of floor distance: " + String.valueOf(motorEncoderEventsPerInch));
        RobotLog.i("Maximum power to motors: " + String.valueOf(maxPower));
        RobotLog.i("Base wheel directions: " + Arrays.toString(getMotorConfigurations(TravelDirection.base)));

        RobotLog.i("*******  *******");

        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);

        setTravelDirection(TravelDirection.base);

        // set motor power to 0.
        stop();
    }

    /**
     * Remove instances of motors, travel directions, speeds
     */
    @Deprecated
    public void cleanup() {
        stop();
        motorConfigurations.clear();
        driveSpeedConfigurations.clear();
        // wheelMotors = null; // unnecessary, GC will handle this for us.
    }

    /**
     * Closes this device
     */
    @Override
    public void close() {
        stop();
        for (DcMotor motor :
                wheelMotors) {
            motor.close();
        }
        motorConfigurations.clear();
        driveSpeedConfigurations.clear();
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return "MMMDrivebase";
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return null;
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    @Override
    public int getVersion() {
        return 0;
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        initMotors();
    }

    /**
     * Set the runmode on all motors.
     * @param runMode how the motors should run - use the enum.
     * @return boolean success
     */
    @ExportToBlocks(
            comment = "Set all motors to the same runmode",
            tooltip = "Use DcMotor RunModes",
            color = 200,
            heading = "drivebase.setRunMode",
            parameterLabels = {"runMode"}
    )
    public boolean setRunMode(DcMotor.RunMode runMode) {
        try {
            DcMotor[] motors = getMotors();
            for (DcMotor motor : motors) {
                motor.setMode(runMode);
            }
            return true;
        }
        catch (Exception e) {
            RobotLog.i(e.getMessage());
            return false;
        }
    }

    /**
     * Set the zero power behavior on all motors.
     * @param behavior zero power behavior - use the enum
     * @return boolean success
     */
    @ExportToBlocks(
            comment = "Set all motors to the same stop mode",
            tooltip = "Use DcMotor Zero Power Behaviors",
            color = 200,
            heading = "drivebase.setStopMode",
            parameterLabels = {"zeroPowerBehavior"}
    )
    public boolean setStopMode(DcMotor.ZeroPowerBehavior behavior) {
        try {
            for (DcMotor motor : getMotors()) {
                motor.setZeroPowerBehavior(behavior);
            }
            return true;
        }
        catch (Exception e) {
            RobotLog.i(e.getMessage());
            return false;
        }
    }

    /**
     * Returns the encoder values for all motors.
     * @return current positions of all motors, in order configured.
     */
    @ExportToBlocks(
            heading = "drivebase.getEncoderPositions",
            comment = "get encoder positions for all wheels",
            tooltip = "Returns an array of integers, which are the encoder positions for each wheel, in order.",
            color = 200)
    public int[] getEncoderPositions() {
        DcMotor[] motors = getMotors();
        int[] positions = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            positions[i] = motors[i].getCurrentPosition();
        }
        return positions;
    }

    /**
     * Set the target position tolerance on each motor.
     * @param tolerance position tolerance
     * @return position tolerance from each motor, in order configured.
     */
    @ExportToBlocks(
            heading = "drivebase.setPositionTolerance",
            comment = "Sets the tolerance on all drive motors",
            tooltip = "When running with encoders, set the tolerance.",
            color = 200)
    public int[] setPositionTolerance(int tolerance) {
        // set position tolerance
        DcMotor[] motors = getMotors();
        int[] tolerances = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            ((DcMotorEx) motors[i]).setTargetPositionTolerance(tolerance);
            tolerances[i] = ((DcMotorEx) motors[i]).getTargetPositionTolerance();
        }
        return tolerances;
    }

    /**
     * Returns the target position tolerance of each motor as an integer array.
     * This probably is unnecessary.  We can remove it if so.
     * @return position tolerance from each motor, in order configured.
     */
    @ExportToBlocks(
            heading = "drivebase.getPositionTolerance",
            comment = "Gets the tolerance for all drive motors",
            tooltip = "When running with encoders, set the tolerance.",
            color = 200)
    public int[] getPositionTolerance() {
        // set position tolerance
        DcMotor[] motors = getMotors();
        int[] tolerances = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            tolerances[i] = ((DcMotorEx) motors[i]).getTargetPositionTolerance();
        }
        return tolerances;
    }

    /**
     * Returns the current position of each motor as an integer array.
     */
    @Deprecated
    public int[] checkMotorPositions() {
        DcMotor[] motors = getMotors();
        int[] positions = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            positions[i] = motors[i].getCurrentPosition();
        }
        return positions;
    }

    /**
     * Stop each motor if it has reached its target.
     * @param targetTicks
     * @param tolerance
     * @return
     */
    @Deprecated
    public boolean[] stopOnTicks(int[] targetTicks, int tolerance) {
        int[] positions = getEncoderPositions();
        DcMotor[] motors = getMotors();
        // track which motors are stopped by this
        boolean[] isStopped = new boolean[motors.length];
        Arrays.fill(isStopped, false);
        for (int i = 0; i < motors.length; i++) {
          if (positions[i] >= (targetTicks[i] + tolerance)) {
              motors[i].setPower(0);
              isStopped[i] = true;
          }
        }
        return isStopped;
    }

    /**
     * Set the direction of travel to one of the predefined configurations.
     * @param travelDirection which way should the bot go?
     * @return  success
     */
    public boolean setTravelDirection(TravelDirection travelDirection) {
        // our motor configuration
        try {
            DcMotor[] motors = getMotors();
            DcMotorSimple.Direction[] directions = getMotorConfigurations(travelDirection);
            if (directions == null) {
                stop();
                return false;
            }
            for (int i = 0; i < motors.length; i++) {
                motors[i].setDirection(directions[i]);
            }
            return true;
        }
        catch (Exception e) {
            stop();
            return false;
        }
    }

    public boolean setMotorDirections(DcMotorSimple.Direction[] directions) {
        try {
            DcMotor[] motors = getMotors();
            for (int i = 0; i < motors.length; i++) {
                motors[i].setDirection(directions[i]);
            }
            return true;
        } catch (Exception ex) {
            RobotLog.e(ex.getMessage());
        }
        return false;
    }

    public DcMotorSimple.Direction[] getMotorDirections() {
        try {
            DcMotor[] motors = getMotors();
            DcMotorSimple.Direction[] directions = new DcMotorSimple.Direction[motors.length];
            for (int i = 0; i < motors.length; i++) {
                directions[i] = motors[i].getDirection();
            }
            return directions;
        } catch (Exception ex) {
            RobotLog.e(ex.getMessage());
        }
        return null;
    }

    public TravelDirection getTravelDirection() {
        try {
            DcMotor[] motors = getMotors();
            DcMotorSimple.Direction[] directions = new DcMotorSimple.Direction[motors.length];
            for (int i = 0; i < motors.length; i++) {
                directions[i] = motors[i].getDirection();
            }
            for (TravelDirection k: this.motorConfigurations.keySet()) {
                DcMotorSimple.Direction[] config = getMotorConfigurations(k);
                boolean match = true;
                for (int i = 0; i < directions.length; i++) {
                    match = match && (config[i] == directions[i]);
                }
                if (match) {
                    return k;
                }
            }
        } catch (Exception ex) {
            RobotLog.e(ex.getMessage());
        }
        return null;
    }

    @ExportToBlocks(
            heading = "drivebase.isBusy",
            comment = "If any drive motors are running, the drivebase is busy.",
            tooltip = "If it's not stopped, it's busy.",
            color = 200)
    public boolean isBusy() {
        DcMotor[] motors = getMotors();
        for (DcMotor motor : motors) {
            if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                if (motor.isBusy()) {
                    return true;
                }
            } else {
                if ((motor.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) && (motor.getPower() != 0)) {
                    //RobotLog.i(motor.getDeviceName() + " is busy - power is " + String.valueOf(motor.getPower()));
                    return true;
                }
            }
        }
        //RobotLog.i("Motors are not busy.");
        return false;
    }

    /**
     * Stop all motors - set power to 0.
     * @return power
     */
    @ExportToBlocks(
            heading = "drivebase.stop",
            comment = "Stops all drive motors",
            tooltip = "Sets the power on all motors to 0.",
            color = 200)
    public double stop()  {
        DcMotor[] motors = getMotors();
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
        return 0;
    }

    /**
     * Is the drivebase really stopped, by using the runmode STOP_AND_RESET_ENCODER?
     * @return
     */
    @ExportToBlocks(
            heading = "drivebase.isStopped",
            comment = "Is it stopped?",
            tooltip = "If all motors are at runmode STOP, it's stopped.",
            color = 200)
    public boolean isStopped() {
        DcMotor[] motors = getMotors();
        boolean stopped = true;
        for (DcMotor motor : motors) {
            stopped = stopped && (motor.getMode() == DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        return stopped;
    }

    /**
     * Go in one of the set directions at the set speed.  Stops on exception.
     * @param direction which way should the bot go?
     * @param driveSpeed how fast?
     * @return the power to the motors
     */
    @ExportToBlocks(
            heading = "drivebase.go",
            comment = "Goes in a specified direction.",
            tooltip = "Sets the power appropriate for the direction the motor is set to.",
            color = 200,
            parameterLabels = { "direction", "driveSpeed" }
    )
    public double go(TravelDirection direction, DriveSpeed driveSpeed)  {
        double power = getDriveSpeedPower(driveSpeed);
        return go(direction, power);
    }

    /**
     * Go in one of the set directions at the set power. Stops on exception.
     * @param direction  which way should the bot go?
     * @param power power value for the motors
     * @return the power to the motors
     */
    public double go(TravelDirection direction, double power) {
        DcMotorSimple.Direction[] directions = getMotorConfigurations(direction);
        return go(directions, power);
    }

    /**
     * Send the motors in any given direction at the set power.  Stops on exception.
     * @param directions  which way should each motor turn?
     * @param power power value for the motors
     * @return the power to the motors
     */
    public double go(DcMotorSimple.Direction[] directions, double power) {
        try {
            if (directions == null) {
                // we asked for a direction we haven't defined.
                RobotLog.i("DriveBase::go: Cannot go in an undefined direction.");
                return stop();
            }
            RobotLog.i("Power: " + String.valueOf(power) + " Directions: " + Arrays.toString(directions));
            DcMotor[] motors = getMotors();
            double motorPower = power;
            for (int i = 0; i < motors.length; i++) {
                // motors should always be set to the base direction.
                // power varies by the directions given.  If forward, positive.
                // if reverse, negative.
                motorPower = power * (directions[i] == DcMotorSimple.Direction.FORWARD ? 1 : -1);
                motors[i].setPower(motorPower);
            }
            return power;
        }
        catch (Exception e) {
            RobotLog.i(e.getMessage());
            return stop();
        }
    }

    /**
     * Set power levels on each motor separately.  Allows finer navigational control of travel paths.
     * Useful especially for a game controller interface.
     * @param powerLevels power level for motor, in order as configured.
     * @return the power level of the first motor.
     */
    public double go(double[] powerLevels) {
        DcMotor[] motors = getMotors();

        // if there aren't enough power levels in the array, add 0 on the end.
        if (powerLevels.length < motors.length) {
            stop();
            throw new ArrayIndexOutOfBoundsException("There are more motors than power levels specified.");
        }

        // set each motor in the array to its pair in the power array.
        for (int i  = 0; i < motors.length; i++) {
            motors[i].setPower(powerLevels[i]);
        }

        // return the first motor's power level.
        return powerLevels.length > 0 ? powerLevels[0] : 0;
    }

    /**
     * Set all motors to a powerlevel.  We assume that a direction and runmode are already set.
     * @param powerLevel
     * @return
     */
    public double go(double powerLevel) {
        DcMotor[] motors = getMotors();
        for (int i = 0; i  < motors.length; i++) {
            motors[i].setPower(powerLevel);
        }
        return powerLevel;
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param travelDirection which way should the bot go?
     * @param power the power to each motor
     * @param encoderTicks the distance each motor  should go, by the encoders
     */
    public double go(TravelDirection travelDirection, double power, int encoderTicks) {
        return go(travelDirection, power, encoderTicks, 0);
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param travelDirection which way should the bot go?
     * @param power the power to each motor
     * @param encoderTicks the distance each motor  should go, by the encoders
     * @param tolerance the tolerance to set on each motor
     */
    public double go(TravelDirection travelDirection, double power, int encoderTicks, int tolerance) {
        DcMotorSimple.Direction[] driveConfiguration = getMotorConfigurations(travelDirection);
        int[] wheelEncoderTicks = new int[driveConfiguration.length];
        DcMotor[] motors = getMotors();
        for (int i = 0; i < motors.length; i++) {
            wheelEncoderTicks[i] = encoderTicks * (driveConfiguration[i] == DcMotorSimple.Direction.FORWARD ? 1 : -1);
        }
        return go(power, wheelEncoderTicks, tolerance);
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param power the power to each motor (we can make this a list if  needed)
     * @param encoderTicks a  list of  encoder tick values,  distances  that each motor should go by the encoders.
     * @param tolerance the tolerance to set on each motor
     */
    public double go(double power, int[] encoderTicks, int tolerance) {
        int [] currentPositions = getEncoderPositions();

        RobotLog.i("Current Positions when driving ticks: " + Arrays.toString(currentPositions));
        RobotLog.i("Power: " + String.valueOf(power) + " Encoder Ticks: " + Arrays.toString(encoderTicks));
        RobotLog.i("Tolerance: " + String.valueOf(tolerance));

        DcMotor[] motors = getMotors();

        // set each motor, based on drive configuration/travel direction
        for (int i = 0; i < motors.length; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setTargetPosition(encoderTicks[i]); //calcTargetPosition(motors[i].getDirection(), currentPositions[i], encoderTicks[i]));
            ((DcMotorEx) motors[i]).setTargetPositionTolerance(tolerance);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(power);

        }

        return power;
    }

    /**
     * Set target position for all motors at once.
     * @param encoderTicks represents the target position for each motor.
     */
    public double setTargetPositions(int encoderTicks) {
        DcMotor[] motors = getMotors();
        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(encoderTicks);
        }
        return encoderTicks;
    }

    /**
     * Get target position for all motors at once.
     */
    public int[] getTargetPositions() {
        DcMotor[] motors = getMotors();
        int[] targetTicks = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            targetTicks[i] = motors[i].getTargetPosition();
        }
        return targetTicks;
    }

    /**
     * Utility method for use in setting encoder values.  Is this correct for reverse directions??
     * NOT NEEDED - this is handled in the DCMotorImpl.
     * @param motorDirection which way is the motor turning?
     * @param currentPosition where are we now?
     * @param encoderTicks how far should we go?
     * @return the encoder value we're shooting for.
     */
    private int calcTargetPosition(DcMotorSimple.Direction motorDirection, int currentPosition, int encoderTicks) {
        switch (motorDirection) {
            case FORWARD:
                return currentPosition + encoderTicks;
            case REVERSE:
                return currentPosition - encoderTicks;
            default:
                return currentPosition;
        }
    }

    /**
     * Returns ticks for the encoder to run when we want to pivot around our own middle.
     * @param theta is the angle in degrees that we should be turning.
     * @return the encoder ticks needed to move the bot that far.
     */
    public double getEncoderValueForRobotPivotAngle(float theta) {
        // calculate the arc of the pivot
        double radius = wheelBaseWidth/2;
        double arcLength = radius * theta * Math.PI / 180; // length in inches

        // now that we have the arcLength, we figure out how many wheel rotations are needed.
        return getEncoderValueForRobotInches(arcLength);
    }

    /**
     * Returns ticks for the encoder to run when we want to cover a distance.
     * @param inchesToTravel how far in inches should the bot go?
     * @return  the encoder ticks needed to move the bot that far.
     */
    public double getEncoderValueForRobotInches(double inchesToTravel) {
        // number of ticks for the encoder
        double ticks = inchesToTravel * motorEncoderEventsPerInch;
        RobotLog.i("Calculated ticks for " + inchesToTravel + " as " + String.valueOf(ticks));
        return ticks;
    }


    /**
     * Returns ticks for the encoder to run when we want to cover a distance.
     * @param mmToTravel how far in inches should the bot go?
     * @return  the encoder ticks needed to move the bot that far.
     */
    public double getEncoderValueForRobotMillimeters(double mmToTravel) {
        double ticks = mmToTravel * motorEncoderEventsPerMM;
        RobotLog.i("Calculated ticks for " + mmToTravel + " as " + String.valueOf(ticks));
        return ticks;
    }

    /**
     * Return the number of ticks difference along the floor between
     * the length between the wheels  when flat and when pitched at angle theta.
     */
    public double getEncoderValueForPitchAngle(double theta) {
        // fr = segment length: front to rear when base is flat
        // rp  = segment length: rear to pivot point
        // fp = segment length: front to pivot point
        // h = segment length: height of pivot point from flat
        // rbp = segment length: rear to pivot along the floor
        // fbp = segment length: front to pivot along the floor

        double rp = inchwormRearMM;
        double fp = inchwormFrontMM;
        double fr = rp + fp;

        double rbp = rp * Math.cos(theta);
        double h = rp * Math.sin(theta);
        double ftheta = Math.asin(h/fp);
        double fbp = fp * Math.cos(ftheta);

        return (fr - (rbp + fbp)) * motorEncoderEventsPerMM;
    }

    public void pitch(int degrees) {
        RobotLog.ww("16221 Drive Base", "Pitching not available for this drive base");
    }
    //TODO: Get Isaac's encoder value for MM routine.
}
