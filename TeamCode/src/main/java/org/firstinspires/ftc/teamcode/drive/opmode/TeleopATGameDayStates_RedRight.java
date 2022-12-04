package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;

/**

 */
//@TeleOp(group = "drive")
@TeleOp(name = "ATTeleOpMode-RedRight")
public class TeleopATGameDayStates_RedRight extends TeleopATGameDayStates {
    @Override
    public void mockupAutonExecution(){
        ATGlobalStorage.autonModeName=ATRobotEnumeration.RED_RIGHT_HIGH_DROP;
        ATGlobalStorage.currentPose=drive.getPoseEstimate();
        ATGlobalStorage.parkingPos=ATRobotEnumeration.PARK2;
        ATGlobalStorage.allianceName=ATRobotEnumeration.RED_ALLIANCE;
        tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, platformAction, hardwareMap);
    }
}
