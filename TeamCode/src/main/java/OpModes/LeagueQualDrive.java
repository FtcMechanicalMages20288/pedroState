package OpModes;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "LeagueQualDrive", group = "TeleOp")

public class LeagueQualDrive extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight, extension;
    private DcMotor intake, rightVerticalMotor, leftVerticalMotor;
    private Servo extendDepo, depoLeft, depoRight, claw, leftHanger, rightHanger, holdChute, intakeTilt, wristClaw;
    private TouchSensor vertSwitch, hortSwitch;
    private ColorSensor colorChute;

    boolean depoSpecMotor = false;
    boolean depoHang = false;
    boolean depoSwing = false;
    boolean dropStall = false;
    boolean upPos = false;
    boolean slideDown = false;
    boolean sampleMotor = false;

    boolean hangBoolean = false;
    boolean transferStall = false;
    boolean closeDelay = false;
    boolean runIntake = false;

    long currentTime = 0;


    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;

    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;
    private double targetValue = 1000;

    boolean rumbled = false;


    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.dcMotor.get("intake");
        extendDepo = hardwareMap.servo.get("extendDepo");
        depoRight = hardwareMap.servo.get("RightDepo");
        depoLeft = hardwareMap.servo.get("LeftDepo");
        claw = hardwareMap.servo.get("claw");
        holdChute = hardwareMap.servo.get("holdChute");

        leftHanger = hardwareMap.servo.get("leftHang");
        rightHanger = hardwareMap.servo.get("rightHang");

        intakeTilt = hardwareMap.servo.get("intakeTilt");
        wristClaw = hardwareMap.servo.get("wristClaw");


        rightVerticalMotor = hardwareMap.dcMotor.get("rightVert");
        leftVerticalMotor = hardwareMap.dcMotor.get("leftVert");
        extension = hardwareMap.dcMotor.get("extension");

        vertSwitch = hardwareMap.touchSensor.get("hortSwitch");//Todo: Changed cause screwed in wiring
        hortSwitch = hardwareMap.touchSensor.get("vertSwitch");

        colorChute = hardwareMap.get(ColorSensor.class, "colorChute");

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //intakeReset();

        intakeInit();

        //holdChute.setPosition(0.41);
        // CloseClaw();
        leftHanger.setPosition(1);
        rightHanger.setPosition(0);
        //intakeTilt.setPosition(0.5);

        //tiltDepo.setPosition(0);


        waitForStart();
        while (opModeIsActive()) {
            tankMecanumMovementController();
            //robotCentricMovementController();


            //Todo: Telemetry of Sensors, Encoders, etc
            getColor();
            colorTelmetry();
            VertTelemetry();
            EncoderTelemetry();


            //0.5


            //Todo: Hangers

            if (gamepad1.a) {
                leftHanger.setPosition(1);
                rightHanger.setPosition(0);
                hangBoolean = false;

            } else if (gamepad1.b) {
                hangBoolean = true;
                leftHanger.setPosition(0.4);
                rightHanger.setPosition(0.6);


            }

            if(gamepad2.right_trigger > 0.3 && hangBoolean){
                rightVerticalMotor.setPower(-1);
                leftVerticalMotor.setPower(1);
                resetPos();
            }
            else if (gamepad2.left_trigger > 0.3 && hangBoolean){
                rightVerticalMotor.setPower(0.8);
                leftVerticalMotor.setPower(-0.8);
            }
            //intakeDown();

                /*
                if(redValue > blueValue){
                    holdChute.setPosition(0.5);
                    telemetry.addData("Red Detected: ", "Poop Chute");
.                    telemetry.update();
                    sleep(100);
                }

                if(blueValue > redValue){
                    holdChute.setPosition(0.43);
                    telemetry.addData("Blue Detected: ", "Hold");
                    telemetry.update();
                    sleep(100);
                }

                 */


            //Todo: Transfer Logic

            /*
            if (!hortSwitch.isPressed()) {
                OpenClaw();
                holdChute.setPosition(0.105);
            }
            else if(hortSwitch.isPressed()) {
                holdChute.setPosition(0.46);
            }

             */



            /*
            if (hortSwitch.isPressed() && !transferStall && !closeDelay) {

                currentTime = System.currentTimeMillis();
                transferStall = true;
                closeDelay = true;

                telemetry.addData("Transfer:", "Ready");
                telemetry.update();

            } else if(!hortSwitch.isPressed()) {
                telemetry.addData("Transfer:", "Not Ready");
                telemetry.update();
                holdChute.setPosition(0.105);//0.13

                runIntake = false;


            }

             */

            if (transferStall) {

                if (System.currentTimeMillis() - currentTime > 1100) {
                    holdChute.setPosition(0.46);
                    transferStall = false;
                }

            } else if (closeDelay) {

                if (System.currentTimeMillis() - currentTime > 750) {
                    CloseClaw();
                    closeDelay = false;
                }

            }

            if (gamepad2.x && !dropStall) {
                CloseClaw();
            }

            //Todo: Intake

            if (gamepad2.a) {
                intake.setPower(-1);
                intakePickPos();
                runIntake = false;

            }
            else if (gamepad2.b) {
                intake.setPower(1);
                intakeSpit();
                runIntake = false;

            } else {
                intake.setPower(0);
            }


            if (gamepad2.y){
                OpenClaw();
            }
            /*
            if (gamepad2.y && !dropStall) {
                currentTime = System.currentTimeMillis();
                dropStall = true;

            }
            if (dropStall) {

                if (System.currentTimeMillis() - currentTime > 1400) {
                    OpenClaw();
                    dropStall = false;
                }

             */

            if(gamepad2.x){
                CloseClaw();
            }



            //Todo: Verts


            boolean vertUp = gamepad2.right_trigger > 0.3;
            boolean vertDown = gamepad2.left_trigger > 0.3;


            if (vertDown) {

                rightVerticalMotor.setPower(-1);
                leftVerticalMotor.setPower(1);
                resetPos();


                /*
                if(leftHanger.getPosition() == 0.4 && rightHanger.getPosition() == 0.6) {
                    rightVerticalMotor.setPower(-0.3);
                    leftVerticalMotor.setPower(0.3);
                }

                else {
                    rightVerticalMotor.setPower(-1);
                    leftVerticalMotor.setPower(1);
                }

                 */

            } else if (vertUp) {
                rightVerticalMotor.setPower(1);
                leftVerticalMotor.setPower(-1);

            } else {
                rightVerticalMotor.setPower(0.13);
                leftVerticalMotor.setPower(-0.13);//0.13
                //change
            }


            boolean BackwardHort = gamepad2.right_bumper;
            boolean ForwardHort = gamepad2.left_bumper;


            if (ForwardHort) { //actually backwards
                extension.setPower(1);
                //intakeSpit();
                backPos();


            } else if (BackwardHort) { //forwards
                extension.setPower(-1);
                backPos();
                //backPos();
            } else {
                extension.setPower(0); //Todo: Consider pressing it in

            }


            boolean vertSpecPick = gamepad2.dpad_left;
            boolean vertSpecHang = gamepad2.dpad_right;
            boolean vertSampleDepo = gamepad2.dpad_up;
            boolean vertReset = gamepad2.dpad_down;


            //Todo: Specimen Hang & PickUp

            if (vertSpecPick && !slideDown) {

                pickSpecPos();

                slideDown = true;

            } else if (vertSpecHang && !depoHang &&  !depoSwing && vertSwitch.isPressed()) {

                CloseClaw();

                currentTime = System.currentTimeMillis();
                depoHang = true;
                depoSwing = true;

            } else if (vertSampleDepo /*!sampleMotor*/ && !upPos) {
                CloseClaw();
                //sampleMotor = true;
                currentTime = System.currentTimeMillis();
                upPos = true;

            } else if (vertReset && !slideDown) {
                resetPos();
                OpenClaw();
                slideDown = true;


            }
            if (slideDown && !rumbled) {
                gamepad2.rumble(500);
                gamepad2.rumble(500);
                rumbled = true;
            } else if (!slideDown && rumbled) {
                rumbled = false;
                gamepad2.stopRumble();
            }

            //Todo: Booleans for delays
            if (depoSpecMotor) {

                if (System.currentTimeMillis() - currentTime < 250) {
                    rightVerticalMotor.setPower(-0.7); //reversed
                    leftVerticalMotor.setPower(0.7);
                    telemetry.addData("Vert Sensor:", "Not Pressed");
                    telemetry.update();

                } else if (vertSwitch.isPressed()) {
                    rightVerticalMotor.setPower(0);
                    leftVerticalMotor.setPower(0);
                    telemetry.addData("Vert Sensor:", "Pressed");
                    telemetry.update();

                    depoSpecMotor = false;
                } else {  //limit switch code can be added here - change to else if
                    rightVerticalMotor.setPower(0);
                    leftVerticalMotor.setPower(0);
                    telemetry.addData("Condition:", "Timeout");
                    telemetry.update();

                    depoSpecMotor = false;
                }

            } else if (depoHang) {
                if (System.currentTimeMillis() - currentTime < 350) {
                    rightVerticalMotor.setPower(1);
                    leftVerticalMotor.setPower(-1);
                } else {  //limit switch code can be added here - change to else if
                    rightVerticalMotor.setPower(0);
                    leftVerticalMotor.setPower(0);
                    depoHang = false;
                }

            } else if (depoSwing) {
                if (System.currentTimeMillis() - currentTime > 350) {
                    specPos();

                    depoSwing = false;
                }
            } else if (upPos) {
                if (System.currentTimeMillis() - currentTime > 400) {
                    depositPos();
                    upPos = false;
                }
            } else if (sampleMotor) {
                if (System.currentTimeMillis() - currentTime < 2400) {
                    rightVerticalMotor.setPower(1);
                    leftVerticalMotor.setPower(-1);
                } else {  //limit switch code can be added here - change to else if
                    rightVerticalMotor.setPower(0);
                    leftVerticalMotor.setPower(0);
                    sampleMotor = false;
                }

            } else if (slideDown) {
                if (!vertSwitch.isPressed()) {
                    rightVerticalMotor.setPower(-0.8);
                    leftVerticalMotor.setPower(0.8);
                } else {
                    rightVerticalMotor.setPower(0); //Todo: change to press down
                    leftVerticalMotor.setPower(0);  //Todo: change to press down

                    slideDown = false;
                }
            }


        }
    }


    //Todo: Attachment Methods

    public void intakeDown() {

        intakeTilt.setPosition(0.55);
    }

    public void intakeReset() {

        intakeTilt.setPosition(0.65);
    }


    public void intakePickPos() {

        intakeTilt.setPosition(0.54);//0.61
    }

    public void intakeInit() {

        intakeTilt.setPosition(0.8);
    }

    public void backPos(){

        intakeTilt.setPosition(0.7);
    }

    public void intakeSpit() {

        intakeTilt.setPosition(0.6);//0.61
    } //0.71


    public void OpenClaw() {

        claw.setPosition(0.46);
    }

    public void CloseClaw() {

        claw.setPosition(0.58);
    }

    public void pickSpecPos() {
        depoRight.setPosition(0.67);
        depoLeft.setPosition(0.33);
        extendDepo.setPosition(0.93);
        wristClaw.setPosition(0.6);
    }

    public void specPos() {
        depoLeft.setPosition(0.9);
        depoRight.setPosition(0.1);
        extendDepo.setPosition(0.93);
        wristClaw.setPosition(0.6);
    }

    public void resetPos() {
        depoRight.setPosition(0.03);
        depoLeft.setPosition(0.97);
        wristClaw.setPosition(0.6); //0.8
        extendDepo.setPosition(0.655);//0.625
    }

    public void depositPos() {
        depoRight.setPosition(0.63);
        depoLeft.setPosition(0.37);
        extendDepo.setPosition(0.71);
    }

    public void getColor() {
        redValue = colorChute.red();
        greenValue = colorChute.green();
        blueValue = colorChute.blue();
        alphaValue = colorChute.alpha();

    }

    public void colorTelmetry() {
        telemetry.addData("red", "%.2f", redValue);
        telemetry.addData("blue", "%.2f", blueValue);
        telemetry.addData("green", "%.2f", greenValue);
        telemetry.addData("alpha", "%.2f", alphaValue);
    }

    public void VertTelemetry() {
        if (vertSwitch.isPressed()) {
            telemetry.addData("Vert Sensor:", "Pressed");
            telemetry.update();
            //sleep(100);

        } else {
            telemetry.addData("Vert Sensor:", "Not Pressed");
            telemetry.update();
        }

    }

    public void EncoderTelemetry() {

        telemetry.addData("leftMot", leftVerticalMotor.getCurrentPosition());
        telemetry.update();

        if (vertSwitch.isPressed()){
            leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void tankMecanumMovementController() {

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (gamepad1.right_bumper) {
            frontLeft.setPower(1);
            backLeft.setPower(-1);
            frontRight.setPower(-1);
            backRight.setPower(1);
        }
        //Strafe left
        else if (gamepad1.left_bumper) {
            frontLeft.setPower(-1);
            backLeft.setPower(1);
            frontRight.setPower(1);
            backRight.setPower(-1);
        } else {
            //regular tank
            right_drivePower = gamepad1.right_stick_y * -1;
            back_left_drivePower = gamepad1.left_stick_y * -1;
            left_drivePower = gamepad1.left_stick_y * -1;
            back_right_drivePower = gamepad1.right_stick_y * -1;


            frontLeft.setPower(left_drivePower);
            frontRight.setPower(right_drivePower);
            backLeft.setPower(left_drivePower);
            backRight.setPower(right_drivePower);
        }

        if (gamepad1.a) {
            frontLeft.setPower(0.4);
            frontRight.setPower(0.4);
            backLeft.setPower(0.4);
            backRight.setPower(0.4);
        }

        if (gamepad1.b) {
            frontLeft.setPower(-0.4);
            frontRight.setPower(-0.4);
            backLeft.setPower(-0.4);
            backRight.setPower(-0.4);
        }

    }


    //Todo: Robot Centric Controller

    private void robotCentricMovementController() {

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //set drive motor zero power behavior to brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double forwardsInput = gamepad1.left_stick_y; //negative bf
        double sidewaysInput = -gamepad1.left_stick_x;
        double steeringInput = -gamepad1.right_stick_x *0.7;
        double speed = 1;


        //calculate motor power
        double leftFrontPower = forwardsInput + sidewaysInput + (steeringInput * speed);
        double rightFrontPower = forwardsInput - sidewaysInput - (steeringInput * speed);
        double leftBackPower = forwardsInput - sidewaysInput + (steeringInput * speed);
        double rightBackPower = forwardsInput + sidewaysInput - (steeringInput * speed);

        //clamp max power
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower = leftBackPower / max;
            rightBackPower = rightBackPower / max;
        }


        //set motor power
        if (gamepad1.right_bumper) {
            frontLeft.setPower(1);
            backLeft.setPower(-1);
            frontRight.setPower(-1);
            backRight.setPower(1);
        }
        //Strafe left
        else if (gamepad1.left_bumper) {
            frontLeft.setPower(-1);
            backLeft.setPower(1);
            frontRight.setPower(1);
            backRight.setPower(-1);
        } else {
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);
        }
    }


}