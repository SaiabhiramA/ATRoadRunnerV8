package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AMotionProfile")

public class TestMotionProfile extends LinearOpMode {
    private DcMotorEx frontLeft, rearLeft, rearRight, frontRight;
    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        boolean x=true;

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested()) {
           if (frontLeft.getCurrentPosition()>=500)
           {
               x=false;
           }
           else if (frontLeft.getCurrentPosition()==0){
               x=true;
           }
          if (x){
                frontLeft.setTargetPosition(500);
                frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                frontLeft.setVelocity(500);

                rearLeft.setTargetPosition(500);
                rearLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rearLeft.setVelocity(500);

                rearRight.setTargetPosition(500);
                rearRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rearRight.setVelocity(500);

                frontRight.setTargetPosition(500);
                frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                frontRight.setVelocity(500);
            } else {
              frontLeft.setTargetPosition(0);
              frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
              frontLeft.setVelocity(500);

              rearLeft.setTargetPosition(0);
              rearLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
              rearLeft.setVelocity(500);

              rearRight.setTargetPosition(0);
              rearRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
              rearRight.setVelocity(500);

              frontRight.setTargetPosition(0);
              frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
              frontRight.setVelocity(500);
            }
        }


    }

}
