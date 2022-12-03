package org.firstinspires.ftc.teamcode.drive.opmode;

//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;


@Autonomous(name = "AMotionProfile")
@Disabled
public class TestMotionProfile extends LinearOpMode {
    /*private DcMotorEx frontLeft, rearLeft, rearRight, frontRight, arm,turntable;
    TrapezoidProfile.Constraints m_constraints =
            new TrapezoidProfile.Constraints(1.75, 0.75);
            //new TrapezoidProfile.Constraints(5, 10);
    ProfiledPIDController m_controller =
            new ProfiledPIDController(1.3, 0.0, 0.7, m_constraints);


    boolean x=true;
    int testCounter=0;*/
    @Override
    public void runOpMode() throws InterruptedException {

       /* frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        turntable = hardwareMap.get(DcMotorEx.class, "turntable");
        MotorEx m_motor=new MotorEx(hardwareMap,"turntable", Motor.GoBILDA.RPM_1620);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        rearLeft.setDirection(DcMotorEx.Direction.REVERSE);
        turntable.setDirection(DcMotorEx.Direction.REVERSE);
        m_motor.setInverted(true);
        m_motor.resetEncoder();

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turntable.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        m_controller.setGoal(500);
        m_motor.setTargetPosition(500);
        m_motor.setRunMode(Motor.RunMode.PositionControl);

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested()) {
            testCounter=testCounter+1;
            //this.testMotionProfilePlatform();
            if (m_motor.getCurrentPosition()>=500)
            {
                x=false;
                m_controller.setGoal(0);
                m_motor.setTargetPosition(0);
                m_motor.setRunMode(Motor.RunMode.PositionControl);
            }
            else if (m_motor.getCurrentPosition()<=0){
                x=true;
                m_controller.setGoal(500);
                m_motor.setTargetPosition(500);
                m_motor.setRunMode(Motor.RunMode.PositionControl);
            }
            if (x){
                m_motor.set(1);
                //m_motor.setVelocity (m_controller.calculate(m_motor.getCurrentPosition()));
            } else {

                m_motor.set(1);
                //m_motor.setVelocity (m_controller.calculate(m_motor.getCurrentPosition(),0));
            }

            m_motor.set(1);
            telemetry.addData("turntable position", m_motor.getCurrentPosition());
            telemetry.addData("X value", x);
            telemetry.addData("testCounter", testCounter);
            telemetry.update();
        }*/
    }

   /* private void testMotionProfilePlatform(){
        if (frontLeft.getCurrentPosition()>=2000)
        {
            x=false;
            m_controller.setGoal(0);
        }
        else if (frontLeft.getCurrentPosition()==0){
            x=true;
            m_controller.setGoal(2000);
        }
        if (x){
            frontLeft.setTargetPosition(2000);
            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //frontLeft.setVelocity(500);

            frontLeft.setVelocity (m_controller.calculate(frontLeft.getCurrentPosition()));


            rearLeft.setTargetPosition(2000);
            rearLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //rearLeft.setVelocity(500);

            rearLeft.setVelocity(m_controller.calculate(rearLeft.getCurrentPosition()));


            rearRight.setTargetPosition(2000);
            rearRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //rearRight.setVelocity(500);

            rearRight.setVelocity(m_controller.calculate(rearRight.getCurrentPosition()));


            frontRight.setTargetPosition(2000);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //frontRight.setVelocity(500);

            frontRight.setVelocity(m_controller.calculate(frontRight.getCurrentPosition()));

        } else {
            frontLeft.setTargetPosition(0);
            frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //frontLeft.setVelocity(500);

            frontLeft.setVelocity (m_controller.calculate(frontLeft.getCurrentPosition()));


            rearLeft.setTargetPosition(0);
            rearLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //rearLeft.setVelocity(500);

            rearLeft.setVelocity(m_controller.calculate(rearLeft.getCurrentPosition()));


            rearRight.setTargetPosition(0);
            rearRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //rearRight.setVelocity(500);

            rearRight.setVelocity(m_controller.calculate(rearRight.getCurrentPosition()));


            frontRight.setTargetPosition(0);
            frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            //frontRight.setVelocity(500);

            frontRight.setVelocity(m_controller.calculate(frontRight.getCurrentPosition()));
        }
    }


    private void turuntableCode(){
        if (turntable.getCurrentPosition()>=1500)
        {
            x=false;
            //m_controller.setGoal(0);
        }
        else if (turntable.getCurrentPosition()<=0){
            x=true;
            //m_controller.setGoal(1500);
        }
        if (x){
            turntable.setTargetPosition(1500);
            turntable.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            m_controller.setConstraints(m_constraints);
            turntable.setVelocity (m_controller.calculate(turntable.getCurrentPosition(),1500));
        } else {
            turntable.setTargetPosition(0);
            turntable.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            m_controller.setConstraints(m_constraints);
            turntable.setVelocity (m_controller.calculate(turntable.getCurrentPosition(),0));
        }
    }
*/

}
