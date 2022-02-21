package org.firstinspires.ftc.teamcode.drivebase

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.drivebase.DriveBase
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.util.RobotConfig
import org.firstinspires.ftc.teamcode.drivebase.DriveBase.TravelDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple

class ProgrammingBoardDriveBase(opMode: OpMode) : DriveBase(opMode.hardwareMap) {

    override fun initMotorConfigurations() {
        var motorLeftFront = hardwareMap.get(DcMotor::class.java, RobotConfig.CURRENT.name("motor_LeftFront"))
        motors = arrayOf<DcMotor>(motorLeftFront)
        motorConfigurations[TravelDirection.forward] = arrayOf(DcMotorSimple.Direction.FORWARD)
        motorConfigurations[TravelDirection.reverse] = arrayOf(DcMotorSimple.Direction.REVERSE)
        motorConfigurations[TravelDirection.pivotLeft] = arrayOf(DcMotorSimple.Direction.REVERSE)
        motorConfigurations[TravelDirection.pivotRight] = arrayOf(DcMotorSimple.Direction.FORWARD)
        motorConfigurations[TravelDirection.strafeLeft] = arrayOf(DcMotorSimple.Direction.FORWARD)
        motorConfigurations[TravelDirection.strafeLeftBackward] = arrayOf(DcMotorSimple.Direction.REVERSE)
        motorConfigurations[TravelDirection.strafeLeftForward] = arrayOf(DcMotorSimple.Direction.FORWARD)
        motorConfigurations[TravelDirection.strafeRight] = arrayOf(DcMotorSimple.Direction.REVERSE)
        motorConfigurations[TravelDirection.strafeRightBackward] = arrayOf(DcMotorSimple.Direction.FORWARD)
        motorConfigurations[TravelDirection.strafeRightForward] = arrayOf(DcMotorSimple.Direction.REVERSE)
        motorConfigurations[TravelDirection.pitch] = arrayOf(DcMotorSimple.Direction.FORWARD)
        // super.initMotorConfigurations();
    }
}