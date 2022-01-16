package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.MMMFreightFrenzyOpMode;

public class FreightFrenzyAutonomous extends MMMFreightFrenzyOpMode {
    private static int startingPosition = 0;

    /**
     * we may need different opModes for different starting positions.
     * @return the starting position for this mode.
     */
    public int getStartingPosition() {
        return startingPosition;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        waitForStart();
        runtime.reset();
        driveBase.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int elementPosition = 0;
        if (opModeIsActive() && !isStopRequested()) {
            RobotLog.i("Reading bar code");
            elementPosition = readBarCode(getStartingPosition());
        }

        if (opModeIsActive() && !isStopRequested()) {
            RobotLog.i("Delivering pre-loaded freight");
            deliverFreight(getStartingPosition(), elementPosition);
        }
    }

    /**
     * Reads the "bar code" on the field by locating the custom element.
     * @param startingPosition The starting position of the robot on the field
     * @return position 1, 2, 3.
     */
    private int readBarCode(int startingPosition) {
        //TODO: use Vision to locate a custom element. We need our starting position to judge where the barcode should be.
        // once we have found our element, return its position.
        // Note that we will need to abort if the opMode is stopped while this is executing.
        // We may want to add a timeout to this just in case TensorFlow takes too long.
        return 1;
    }

    /**
     * Delivers the preloaded freight from the starting position.
     * @param startingPosition The starting position of the robot on the field
     * @return success.
     */
    private boolean deliverFreight(int startingPosition, int elementPosition) {
        while (opModeIsActive() && !isStopRequested()) {
            //TODO: navigate to the alliance shipping hub and deliver the preloaded freight to the correct level.

        }
        // if we fail to complete this task, return false. Otherwise, return true.
        return true;
    }
}
