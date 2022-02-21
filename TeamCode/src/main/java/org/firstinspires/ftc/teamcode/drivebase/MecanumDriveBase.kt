package org.firstinspires.ftc.teamcode.drivebase

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.drivebase.DriveBase
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.RobotConfig
import org.firstinspires.ftc.teamcode.drivebase.DriveBase.TravelDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple

/**
 * Use this superclass to control a drive base with four mecanum wheels, one at each corner.
 */
open class MecanumDriveBase(opMode: OpMode) : DriveBase(opMode.hardwareMap) {

    override fun initMotorConfigurations() {
        var motorLeftFront = hardwareMap.dcMotor[RobotConfig.CURRENT.name("motor_LeftFront")]
        var motorRightFront = hardwareMap.dcMotor[RobotConfig.CURRENT.name("motor_RightFront")]
        var motorLeftRear = hardwareMap.dcMotor[RobotConfig.CURRENT.name("motor_LeftRear")]
        var motorRightRear = hardwareMap.dcMotor[RobotConfig.CURRENT.name("motor_RightRear")]
        motors = arrayOf<DcMotor>(
                motorLeftFront, motorRightFront,
                motorLeftRear, motorRightRear
        )

        // We use the motorConfigurations to determine whether the power and encoder
        // values should be positive or negative when attempting any particular direction.
        // The directions are typically used in Autonomous programming.
        // TeleOp should use the TravelDirection.forward configuration.
        motorConfigurations[TravelDirection.base] = arrayOf(
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE
        )
        motorConfigurations[TravelDirection.forward] = arrayOf(
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD
        )
        motorConfigurations[TravelDirection.reverse] = arrayOf(
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE
        )
        motorConfigurations[TravelDirection.pivotLeft] = arrayOf(
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD
        )
        motorConfigurations[TravelDirection.pivotRight] = arrayOf(
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE
        )
        motorConfigurations[TravelDirection.strafeLeftForward] = arrayOf<DcMotorSimple.Direction?>(
                null, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD, null
        )
        motorConfigurations[TravelDirection.strafeLeft] = arrayOf(
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE
        )
        motorConfigurations[TravelDirection.strafeLeftBackward] = arrayOf<DcMotorSimple.Direction?>(
                DcMotorSimple.Direction.REVERSE, null,
                null, DcMotorSimple.Direction.REVERSE
        )
        motorConfigurations[TravelDirection.strafeRightForward] = arrayOf<DcMotorSimple.Direction?>(
                DcMotorSimple.Direction.FORWARD, null,
                null, DcMotorSimple.Direction.FORWARD
        )
        motorConfigurations[TravelDirection.strafeRight] = arrayOf(
                DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD
        )
        motorConfigurations[TravelDirection.strafeRightBackward] = arrayOf<DcMotorSimple.Direction?>(
                null, DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE, null
        )
        //        motorConfigurations.put(
//            TravelDirection.pitch,
//            new Direction[]{
//                null, null,
//                leftMotorDirection(Direction.FORWARD), rightMotorDirection(Direction.REVERSE)
//            }
//        );

        // super.initMotorConfigurations();
    }

    companion object {
        // for consistency, always use these methods to translate the actual motor direction
        // for the different sides of the bot.
        // if the motor orientation changes on the hardware, adjust these methods to suit.
        private fun rightMotorDirection(externalDirection: DcMotorSimple.Direction): DcMotorSimple.Direction {
            return externalDirection
        }

        // reverse the motor direction for the motors on the right side of the bot.
        private fun leftMotorDirection(externalDirection: DcMotorSimple.Direction): DcMotorSimple.Direction {
            return externalDirection
            //        if (externalDirection == Direction.FORWARD) {
//            return Direction.REVERSE;
//        } else {
//            return Direction.FORWARD;
//        }
        }
    }
}