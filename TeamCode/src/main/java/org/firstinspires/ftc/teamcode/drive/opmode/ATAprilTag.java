
        /*
         * Copyright (c) 2021 OpenFTC Team
         *
         * Permission is hereby granted, free of charge, to any person obtaining a copy
         * of this software and associated documentation files (the "Software"), to deal
         * in the Software without restriction, including without limitation the rights
         * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
         * copies of the Software, and to permit persons to whom the Software is
         * furnished to do so, subject to the following conditions:
         *
         * The above copyright notice and this permission notice shall be included in all
         * copies or substantial portions of the Software.
         * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
         * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
         * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
         * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
         * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
         * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
         * SOFTWARE.
         */

        package org.firstinspires.ftc.teamcode.drive.opmode;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
        import org.openftc.apriltag.AprilTagDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;

        import java.util.ArrayList;


        public class ATAprilTag
        {
            int count= 0 ;
            private HardwareMap hardwareMap;
            private ATRobotEnumeration robotMode;
            private Telemetry telemetry;
            ATRobotEnumeration parkZone ;
            OpenCvCamera camera;

            TestPipeline aprilTagDetectionPipeline;

            static final double FEET_PER_METER = 3.28084;

            // Lens intrinsics
            // UNITS ARE PIXELS
            // NOTE: this calibration is for the C920 webcam at 800x448.
            // You will need to do your own calibration for other configurations!
            double fx = 578.272;
            double fy = 578.272;
            double cx = 402.145;
            double cy = 221.506;

            // UNITS ARE METERS
            double tagsize = 0.166;
            int LEFT =1;
            int MIDDLE = 2;
            int RIGHT = 3;

            int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

            AprilTagDetection tagOfInterest = null;


            public void initalizeTensorFlow(HardwareMap hwmap , Telemetry ATTelemetry, ATRobotEnumeration rMode) throws InterruptedException {
                // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
                // first.
                hardwareMap = hwmap;
                telemetry = ATTelemetry;
                robotMode = rMode;
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

                aprilTagDetectionPipeline = new TestPipeline(tagsize, fx, fy, cx, cy);

                camera.setPipeline(aprilTagDetectionPipeline);
                camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                         camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                        telemetry.addData("CAmera open", "OPen");
                    }

                    @Override
                    public void onError(int errorCode) {
                    }
                });

            }
            public ATRobotEnumeration detectObjectLabel() throws InterruptedException {


                /*
                 * The INIT-loop:
                 * This REPLACES waitForStart!
                 */

              //  while ( RedAllianceLeftHighDrop.startprog = false && count < 100000)
              //  while (  count < 250)
                //{

                    ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                  // if(currentDetections!=null) {telemetry.addLine("DTECtions: " + currentDetections.size() );}
                    if(currentDetections!=null && currentDetections.size() != 0)
                    {
                        for(AprilTagDetection tag : currentDetections)
                        {
                            telemetry.addLine("FOR LOOP: " + "LOOP" );
                            if(tag.id == LEFT )
                            {
                                telemetry.addLine("PARK ZONE APRIL TAG: " + "PARK1" );
                                parkZone = ATRobotEnumeration.PARK1;


                            } else if(tag.id == MIDDLE){
                            telemetry.addLine("PARK ZONE APRIL TAG: " + "PARK2" );
                                parkZone = ATRobotEnumeration.PARK2;;

                            } else if(tag.id== RIGHT){
                            telemetry.addLine("PARK ZONE APRIL TAG: " + "PARK3" );
                                parkZone = ATRobotEnumeration.PARK3;;

                            } else {
                                parkZone = ATRobotEnumeration.SUBSTATION;
                            }
                            //Thread.sleep(1000);
                        }
                    }

               // }    // ENd While

                return parkZone;
            }
        }
