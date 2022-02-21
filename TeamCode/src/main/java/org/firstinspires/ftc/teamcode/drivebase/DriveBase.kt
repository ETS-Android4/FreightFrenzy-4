package org.firstinspires.ftc.teamcode.drivebase

import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Subassembly
import com.qualcomm.robotcore.hardware.HardwareDevice
import org.firstinspires.ftc.teamcode.util.Conversions
import org.firstinspires.ftc.teamcode.drivebase.DriveBase.TravelDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.drivebase.DriveBase.DriveSpeed
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import java.lang.Exception
import java.util.*
import kotlin.jvm.JvmOverloads

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
abstract class DriveBase(protected var hardwareMap: HardwareMap) : Subassembly, HardwareDevice {
    /**
     * Constants to represent drive speeds that are useful.  Set power values for each in initDriveSpeedConfiguration().
     */
    enum class DriveSpeed {
        STOP, ANGLE_CORRECTION, SLOW, FAST, MAX
    }

    // Wheel diameter tells us how far it will travel.
    // Metric
    var wheelDiameterMM = 96.0
    var wheelRotationDistanceMM = Math.PI * wheelDiameterMM
    var motorRotationsPerMM = 1 / wheelRotationDistanceMM

    // English
    var wheelDiameterInches = Conversions.mmToInches(wheelDiameterMM)
    var wheelRotationDistanceInches = Math.PI * wheelDiameterInches
    var motorRotationsPerInch = 1 / wheelRotationDistanceInches
    var wheelBaseWidth = 16.0 // inches
    var wheelBaseLengthMM = 360.0
    var inchwormFrontMM = 216.0 // front to pivot point
    var inchwormRearMM = 144.0 // rear to pivot point

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-223-rpm-3-3-5v-encoder/
    @JvmField
    var motorEncoderEventsPerRotation = 753.2
    var motorEncoderEventsPerInch = motorRotationsPerInch * motorEncoderEventsPerRotation
    @JvmField
    var motorEncoderEventsPerMM = motorRotationsPerMM * motorEncoderEventsPerRotation
    var maxPower = 1.0 // range of 0 to 1

    /**
     * A predefined set of travel directions. Set motor configurations for each direction in initMotorConfigurations().
     */
    enum class TravelDirection {
        base, forward, reverse, pivotLeft, pivotRight, strafeLeft, strafeLeftForward, strafeLeftBackward, strafeRight, strafeRightForward, strafeRightBackward, pitch
    }

    @JvmField
    protected var motorConfigurations = Hashtable<TravelDirection, Array<DcMotorSimple.Direction?>>()

    /**
     * Configure motors in initMotorConfigurations.
     * @return the series, in order as configured, of motor objects that are relevant to this drivebase.
     */
    var motors = arrayOf<DcMotor>()
        protected set

    /**
     * Add hashtable entries to Hashtable&lt;TravelDirection, DcMotorSimple.Direction[]&gt; motorConfigurations,
     * representing the directions each motor must turn to send the bot in a given direction.
     * This is mostly useful for autonomous driving; however, teleOp may wish to use these to understand
     * motor directions when handling user input from a gamepad.
     */
    protected open fun initMotorConfigurations() {
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
    fun getMotorConfigurations(travelDirection: TravelDirection): Array<DcMotorSimple.Direction?> {
        return motorConfigurations.getOrDefault(travelDirection, arrayOf(DcMotorSimple.Direction.FORWARD))
    }

    protected var driveSpeedConfigurations = Hashtable<DriveSpeed, Double>()

    /**
     * Set up the standard power constants for the different drive speeds.
     */
    protected fun initDriveSpeedConfigurations() {
        // add hashtable entries to the driveSpeedConfigurations here, based on the configured motors and wheels.
        driveSpeedConfigurations[DriveSpeed.STOP] = 0.0
        driveSpeedConfigurations[DriveSpeed.SLOW] = 0.2
        driveSpeedConfigurations[DriveSpeed.FAST] = 0.8
        driveSpeedConfigurations[DriveSpeed.MAX] = 1.0
    }

    /**
     * Returns the power value for the given drive speed, or null.
     * @param driveSpeed how fast we should go - use the enum
     * @return the current power for the drive speed
     */
    fun getDriveSpeedPower(driveSpeed: DriveSpeed): Double {
        return driveSpeedConfigurations.getOrDefault(driveSpeed, 0.toDouble())
    }

    /**
     * Initialize all motors to run using encoders.
     * Set travel to forward and power to  0.
     */
    @ExportToBlocks(heading = "drivebase.initMotors", comment = "Initialize all drive motors", tooltip = "Stop, reset, brake, set to default directions, set power to 0", color = 200)
    fun initMotors() {
        initMotorConfigurations()
        initDriveSpeedConfigurations()

        // Output all configuration information for the drive base.
        RobotLog.i("*******  Drive Base Configuration *******")
        RobotLog.i("Wheel diameter (inches): $wheelDiameterInches")
        RobotLog.i("Floor distance for one wheel rotation (inches): $wheelRotationDistanceInches")
        RobotLog.i("Rotations per inch of floor distance: $motorRotationsPerInch")
        RobotLog.i("Wheel base width (inches): $wheelBaseWidth")
        RobotLog.i("Motor encoder events per wheel rotation: $motorEncoderEventsPerRotation")
        RobotLog.i("Motor encoder events per inch of floor distance: $motorEncoderEventsPerInch")
        RobotLog.i("Maximum power to motors: $maxPower")
        RobotLog.i("Base wheel directions: " + Arrays.toString(getMotorConfigurations(TravelDirection.base)))
        RobotLog.i("*******  *******")
        setRunMode(RunMode.STOP_AND_RESET_ENCODER)
        setStopMode(ZeroPowerBehavior.BRAKE)
        setTravelDirection(TravelDirection.base)

        // set motor power to 0.
        stop()
    }

    /**
     * Remove instances of motors, travel directions, speeds
     */
    @Deprecated("")
    fun cleanup() {
        stop()
        motorConfigurations.clear()
        driveSpeedConfigurations.clear()
        // wheelMotors = null; // unnecessary, GC will handle this for us.
    }

    /**
     * Closes this device
     */
    override fun close() {
        stop()
        for (motor in motors) {
            motor.close()
        }
        motorConfigurations.clear()
        driveSpeedConfigurations.clear()
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    override fun getManufacturer(): Manufacturer {
        return Manufacturer.Other
    }

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    override fun getDeviceName(): String {
        return "MMMDrivebase"
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    override fun getConnectionInfo(): String {
        return "dunno"
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    override fun getVersion(): Int {
        return 0
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    override fun resetDeviceConfigurationForOpMode() {
        initMotors()
    }

    /**
     * Set the runmode on all motors.
     * @param runMode how the motors should run - use the enum.
     * @return boolean success
     */
    @ExportToBlocks(comment = "Set all motors to the same runmode", tooltip = "Use DcMotor RunModes", color = 200, heading = "drivebase.setRunMode", parameterLabels = ["runMode"])
    fun setRunMode(runMode: RunMode?): Boolean {
        return try {
            val motors = motors
            for (motor in motors) {
                motor.mode = runMode
            }
            true
        } catch (e: Exception) {
            RobotLog.i(e.message)
            false
        }
    }

    /**
     * Set the zero power behavior on all motors.
     * @param behavior zero power behavior - use the enum
     * @return boolean success
     */
    @ExportToBlocks(comment = "Set all motors to the same stop mode", tooltip = "Use DcMotor Zero Power Behaviors", color = 200, heading = "drivebase.setStopMode", parameterLabels = ["zeroPowerBehavior"])
    fun setStopMode(behavior: ZeroPowerBehavior?): Boolean {
        return try {
            for (motor in motors) {
                motor.zeroPowerBehavior = behavior
            }
            true
        } catch (e: Exception) {
            RobotLog.i(e.message)
            false
        }
    }

    /**
     * Returns the encoder values for all motors.
     * @return current positions of all motors, in order configured.
     */
    @get:ExportToBlocks(heading = "drivebase.getEncoderPositions", comment = "get encoder positions for all wheels", tooltip = "Returns an array of integers, which are the encoder positions for each wheel, in order.", color = 200)
    val encoderPositions: IntArray
        get() {
            val motors = motors
            val positions = IntArray(motors.size)
            for (i in motors.indices) {
                positions[i] = motors[i].currentPosition
            }
            return positions
        }

    /**
     * Set the target position tolerance on each motor.
     * @param tolerance position tolerance
     * @return position tolerance from each motor, in order configured.
     */
    @ExportToBlocks(heading = "drivebase.setPositionTolerance", comment = "Sets the tolerance on all drive motors", tooltip = "When running with encoders, set the tolerance.", color = 200)
    fun setPositionTolerance(tolerance: Int): IntArray {
        // set position tolerance
        val motors = motors
        val tolerances = IntArray(motors.size)
        for (i in motors.indices) {
            (motors[i] as DcMotorEx).targetPositionTolerance = tolerance
            tolerances[i] = (motors[i] as DcMotorEx).targetPositionTolerance
        }
        return tolerances
    }// set position tolerance

    /**
     * Returns the target position tolerance of each motor as an integer array.
     * This probably is unnecessary.  We can remove it if so.
     * @return position tolerance from each motor, in order configured.
     */
    @get:ExportToBlocks(heading = "drivebase.getPositionTolerance", comment = "Gets the tolerance for all drive motors", tooltip = "When running with encoders, set the tolerance.", color = 200)
    val positionTolerance: IntArray
        get() {
            // set position tolerance
            val motors = motors
            val tolerances = IntArray(motors.size)
            for (i in motors.indices) {
                tolerances[i] = (motors[i] as DcMotorEx).targetPositionTolerance
            }
            return tolerances
        }

    /**
     * Returns the current position of each motor as an integer array.
     */
    @Deprecated("")
    fun checkMotorPositions(): IntArray {
        val motors = motors
        val positions = IntArray(motors.size)
        for (i in motors.indices) {
            positions[i] = motors[i].currentPosition
        }
        return positions
    }

    /**
     * Stop each motor if it has reached its target.
     * @param targetTicks
     * @param tolerance
     * @return
     */
    @Deprecated("")
    fun stopOnTicks(targetTicks: IntArray, tolerance: Int): BooleanArray {
        val positions = encoderPositions
        val motors = motors
        // track which motors are stopped by this
        val isStopped = BooleanArray(motors.size)
        Arrays.fill(isStopped, false)
        for (i in motors.indices) {
            if (positions[i] >= targetTicks[i] + tolerance) {
                motors[i].power = 0.0
                isStopped[i] = true
            }
        }
        return isStopped
    }

    /**
     * Set the direction of travel to one of the predefined configurations.
     * @param travelDirection which way should the bot go?
     * @return  success
     */
    fun setTravelDirection(travelDirection: TravelDirection): Boolean {
        // our motor configuration
        return try {
            val motors = motors
            val directions = getMotorConfigurations(travelDirection)
            if (directions == null) {
                stop()
                return false
            }
            for (i in motors.indices) {
                motors[i].direction = directions[i]
            }
            true
        } catch (e: Exception) {
            stop()
            false
        }
    }

    fun setMotorDirections(directions: Array<DcMotorSimple.Direction?>): Boolean {
        try {
            val motors = motors
            for (i in motors.indices) {
                motors[i].direction = directions[i]
            }
            return true
        } catch (ex: Exception) {
            RobotLog.e(ex.message)
        }
        return false
    }

    val motorDirections: Array<DcMotorSimple.Direction?>?
        get() {
            try {
                val motors = motors
                val directions = arrayOfNulls<DcMotorSimple.Direction>(motors.size)
                for (i in motors.indices) {
                    directions[i] = motors[i].direction
                }
                return directions
            } catch (ex: Exception) {
                RobotLog.e(ex.message)
            }
            return null
        }
    val travelDirection: TravelDirection?
        get() {
            try {
                val motors = motors
                val directions = arrayOfNulls<DcMotorSimple.Direction>(motors.size)
                for (i in motors.indices) {
                    directions[i] = motors[i].direction
                }
                for (k in motorConfigurations.keys) {
                    val config = getMotorConfigurations(k)
                    var match = true
                    for (i in directions.indices) {
                        match = match && config!![i] == directions[i]
                    }
                    if (match) {
                        return k
                    }
                }
            } catch (ex: Exception) {
                RobotLog.e(ex.message)
            }
            return null
        }

    //RobotLog.i(motor.getDeviceName() + " is busy - power is " + String.valueOf(motor.getPower()));
    //RobotLog.i("Motors are not busy.");
    @get:ExportToBlocks(heading = "drivebase.isBusy", comment = "If any drive motors are running, the drivebase is busy.", tooltip = "If it's not stopped, it's busy.", color = 200)
    val isBusy: Boolean
        get() {
            val motors = motors
            for (motor in motors) {
                if (motor.mode == RunMode.RUN_TO_POSITION) {
                    if (motor.isBusy) {
                        return true
                    }
                } else {
                    if (motor.mode != RunMode.STOP_AND_RESET_ENCODER && motor.power != 0.0) {
                        //RobotLog.i(motor.getDeviceName() + " is busy - power is " + String.valueOf(motor.getPower()));
                        return true
                    }
                }
            }
            //RobotLog.i("Motors are not busy.");
            return false
        }

    /**
     * Stop all motors - set power to 0.
     * @return power
     */
    @ExportToBlocks(heading = "drivebase.stop", comment = "Stops all drive motors", tooltip = "Sets the power on all motors to 0.", color = 200)
    fun stop(): Double {
        val motors = motors
        for (motor in motors) {
            motor.power = 0.0
        }
        return 0.0
    }

    /**
     * Is the drivebase really stopped, by using the runmode STOP_AND_RESET_ENCODER?
     * @return
     */
    @get:ExportToBlocks(heading = "drivebase.isStopped", comment = "Is it stopped?", tooltip = "If all motors are at runmode STOP, it's stopped.", color = 200)
    val isStopped: Boolean
        get() {
            val motors = motors
            var stopped = true
            for (motor in motors) {
                stopped = stopped && motor.mode == RunMode.STOP_AND_RESET_ENCODER
            }
            return stopped
        }

    /**
     * Go in one of the set directions at the set speed.  Stops on exception.
     * @param direction which way should the bot go?
     * @param driveSpeed how fast?
     * @return the power to the motors
     */
    @ExportToBlocks(heading = "drivebase.go", comment = "Goes in a specified direction.", tooltip = "Sets the power appropriate for the direction the motor is set to.", color = 200, parameterLabels = ["direction", "driveSpeed"])
    open fun go(direction: TravelDirection, driveSpeed: DriveSpeed): Double {
        val power = getDriveSpeedPower(driveSpeed)
        return go(direction, power)
    }

    /**
     * Go in one of the set directions at the set power. Stops on exception.
     * @param direction  which way should the bot go?
     * @param power power value for the motors
     * @return the power to the motors
     */
    @JvmOverloads
    fun go(direction: TravelDirection, power: Double): Double {
        val directions = getMotorConfigurations(direction)
        return go(directions, power)
    }

    /**
     * Send the motors in any given direction at the set power.  Stops on exception.
     * @param directions  which way should each motor turn?
     * @param power power value for the motors
     * @return the power to the motors
     */
    @JvmOverloads
    fun go(directions: Array<DcMotorSimple.Direction?>, power: Double): Double {
        return try {
            if (directions == null) {
                // we asked for a direction we haven't defined.
                RobotLog.i("DriveBase::go: Cannot go in an undefined direction.")
                return stop()
            }
            RobotLog.i("Power: " + power.toString() + " Directions: " + Arrays.toString(directions))
            val motors = motors
            var motorPower = power
            for (i in motors.indices) {
                // motors should always be set to the base direction.
                // power varies by the directions given.  If forward, positive.
                // if reverse, negative.
                motorPower = power * if (directions[i] == DcMotorSimple.Direction.FORWARD) 1 else -1
                motors[i].power = motorPower
            }
            power
        } catch (e: Exception) {
            RobotLog.i(e.message)
            stop()
        }
    }

    /**
     * Set power levels on each motor separately.  Allows finer navigational control of travel paths.
     * Useful especially for a game controller interface.
     * @param powerLevels power level for motor, in order as configured.
     * @return the power level of the first motor.
     */
    @JvmOverloads
    fun go(powerLevels: DoubleArray): Double {
        val motors = motors

        // if there aren't enough power levels in the array, add 0 on the end.
        if (powerLevels.size < motors.size) {
            stop()
            throw ArrayIndexOutOfBoundsException("There are more motors than power levels specified.")
        }

        // set each motor in the array to its pair in the power array.
        for (i in motors.indices) {
            motors[i].power = powerLevels[i]
        }

        // return the first motor's power level.
        return if (powerLevels.size > 0) powerLevels[0] else 0.0
    }

    /**
     * Set all motors to a powerlevel.  We assume that a direction and runmode are already set.
     * @param powerLevel
     * @return
     */
    @JvmOverloads
    fun go(powerLevel: Double): Double {
        val motors = motors
        for (i in motors.indices) {
            motors[i].power = powerLevel
        }
        return powerLevel
    }
    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param travelDirection which way should the bot go?
     * @param power the power to each motor
     * @param encoderTicks the distance each motor  should go, by the encoders
     * @param tolerance the tolerance to set on each motor
     */
    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param travelDirection which way should the bot go?
     * @param power the power to each motor
     * @param encoderTicks the distance each motor  should go, by the encoders
     */
    @JvmOverloads
    fun go(travelDirection: TravelDirection, power: Double, encoderTicks: Int, tolerance: Int = 0): Double {
        val driveConfiguration = getMotorConfigurations(travelDirection)
        val wheelEncoderTicks = IntArray(driveConfiguration!!.size)
        val motors = motors
        for (i in motors.indices) {
            wheelEncoderTicks[i] = encoderTicks * if (driveConfiguration[i] == DcMotorSimple.Direction.FORWARD) 1 else -1
        }
        return go(power, wheelEncoderTicks, tolerance)
    }

    /**
     * Drive a number of ticks in a particular direction. Stops on exception.
     * @param power the power to each motor (we can make this a list if  needed)
     * @param encoderTicks a  list of  encoder tick values,  distances  that each motor should go by the encoders.
     * @param tolerance the tolerance to set on each motor
     */
    @JvmOverloads
    fun go(power: Double, encoderTicks: IntArray, tolerance: Int): Double {
        val currentPositions = encoderPositions
        RobotLog.i("Current Positions when driving ticks: " + Arrays.toString(currentPositions))
        RobotLog.i("Power: " + power.toString() + " Encoder Ticks: " + Arrays.toString(encoderTicks))
        RobotLog.i("Tolerance: $tolerance")
        val motors = motors

        // set each motor, based on drive configuration/travel direction
        for (i in motors.indices) {
            motors[i].mode = RunMode.RUN_USING_ENCODER
            motors[i].targetPosition = encoderTicks[i] //calcTargetPosition(motors[i].getDirection(), currentPositions[i], encoderTicks[i]));
            (motors[i] as DcMotorEx).targetPositionTolerance = tolerance
            motors[i].mode = RunMode.RUN_TO_POSITION
            motors[i].power = power
        }
        return power
    }

    /**
     * Set target position for all motors at once.
     * @param encoderTicks represents the target position for each motor.
     */
    fun setTargetPositions(encoderTicks: Int): Double {
        val motors = motors
        for (i in motors.indices) {
            motors[i].targetPosition = encoderTicks
        }
        return encoderTicks.toDouble()
    }

    /**
     * Get target position for all motors at once.
     */
    val targetPositions: IntArray
        get() {
            val motors = motors
            val targetTicks = IntArray(motors.size)
            for (i in motors.indices) {
                targetTicks[i] = motors[i].targetPosition
            }
            return targetTicks
        }

    /**
     * Utility method for use in setting encoder values.  Is this correct for reverse directions??
     * NOT NEEDED - this is handled in the DCMotorImpl.
     * @param motorDirection which way is the motor turning?
     * @param currentPosition where are we now?
     * @param encoderTicks how far should we go?
     * @return the encoder value we're shooting for.
     */
    private fun calcTargetPosition(motorDirection: DcMotorSimple.Direction, currentPosition: Int, encoderTicks: Int): Int {
        return when (motorDirection) {
            DcMotorSimple.Direction.FORWARD -> currentPosition + encoderTicks
            DcMotorSimple.Direction.REVERSE -> currentPosition - encoderTicks
            else -> currentPosition
        }
    }

    /**
     * Returns ticks for the encoder to run when we want to pivot around our own middle.
     * @param theta is the angle in degrees that we should be turning.
     * @return the encoder ticks needed to move the bot that far.
     */
    fun getEncoderValueForRobotPivotAngle(theta: Float): Double {
        // calculate the arc of the pivot
        val radius = wheelBaseWidth / 2
        val arcLength = radius * theta * Math.PI / 180 // length in inches

        // now that we have the arcLength, we figure out how many wheel rotations are needed.
        return getEncoderValueForRobotInches(arcLength)
    }

    /**
     * Returns ticks for the encoder to run when we want to cover a distance.
     * @param inchesToTravel how far in inches should the bot go?
     * @return  the encoder ticks needed to move the bot that far.
     */
    fun getEncoderValueForRobotInches(inchesToTravel: Double): Double {
        // number of ticks for the encoder
        val ticks = inchesToTravel * motorEncoderEventsPerInch
        RobotLog.i("Calculated ticks for $inchesToTravel as $ticks")
        return ticks
    }

    /**
     * Returns ticks for the encoder to run when we want to cover a distance.
     * @param mmToTravel how far in inches should the bot go?
     * @return  the encoder ticks needed to move the bot that far.
     */
    fun getEncoderValueForRobotMillimeters(mmToTravel: Double): Double {
        val ticks = mmToTravel * motorEncoderEventsPerMM
        RobotLog.i("Calculated ticks for $mmToTravel as $ticks")
        return ticks
    }

    /**
     * Return the number of ticks difference along the floor between
     * the length between the wheels  when flat and when pitched at angle theta.
     */
    fun getEncoderValueForPitchAngle(theta: Double): Double {
        // fr = segment length: front to rear when base is flat
        // rp  = segment length: rear to pivot point
        // fp = segment length: front to pivot point
        // h = segment length: height of pivot point from flat
        // rbp = segment length: rear to pivot along the floor
        // fbp = segment length: front to pivot along the floor
        val rp = inchwormRearMM
        val fp = inchwormFrontMM
        val fr = rp + fp
        val rbp = rp * Math.cos(theta)
        val h = rp * Math.sin(theta)
        val ftheta = Math.asin(h / fp)
        val fbp = fp * Math.cos(ftheta)
        return (fr - (rbp + fbp)) * motorEncoderEventsPerMM
    }

    open fun pitch(degrees: Int) {
        RobotLog.ww("16221 Drive Base", "Pitching not available for this drive base")
    } //TODO: Get Isaac's encoder value for MM routine.

    /**
     * When instantiating a DriveBase subclass, pass in the hardware map from the OpMode.
     * @param hwMap the hardware map from the opmode
     */
    init {
        resetDeviceConfigurationForOpMode()
    }
}