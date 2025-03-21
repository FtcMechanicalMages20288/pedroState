package OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Thanos Drive")

public class LeagueQualDrive extends LinearOpMode {

    //DEFINING VARIABLES

    //defining motors
    private DcMotor frontLeft, backLeft, frontRight, backRight; //drive motors
    private DcMotor extension, rightVerticalMotor, leftVerticalMotor; //slide motors
    private DcMotor intake; //intake motor

    //defining servos
    private Servo extendDepo, depoLeft, depoRight; //depo servos
    private Servo claw, wristClaw; //claw servos
    private Servo leftHanger, rightHanger; //hang servos
    private Servo intakeStopper, intakeTilt; //intake servos

    //defining sensors
    private TouchSensor vertSwitch, hortSwitch;
    private ColorSensor intakeColorSensor;
    private DistanceSensor intakeDistanceSensor;

    //defining intake sensor variables
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;
    private double redThreshold = 200;
    private double blueThreshold = 0;
    private double distanceThreshold = 40;
    private int currentSlideResolution;
    boolean blockFound = false;

    private boolean allianceIsBlue = true;
    private boolean specimenToHighBar = false;
    private boolean Rumbled = false;
    private boolean intakeRumbled = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //ASSIGNING ALL HARDWARE

        //intake
        intake = hardwareMap.dcMotor.get("intake");
        intakeStopper = hardwareMap.servo.get("holdChute");
        intakeTilt = hardwareMap.servo.get("intakeTilt");

        //depo
        extendDepo = hardwareMap.servo.get("extendDepo");
        depoRight = hardwareMap.servo.get("RightDepo");
        depoRight.setDirection(Servo.Direction.REVERSE);
        depoLeft = hardwareMap.servo.get("LeftDepo");

        //claw
        claw = hardwareMap.servo.get("claw");
        wristClaw = hardwareMap.servo.get("wristClaw");

        //hang
        leftHanger = hardwareMap.servo.get("leftHang");
        rightHanger = hardwareMap.servo.get("rightHang");

        //slides
        rightVerticalMotor = hardwareMap.dcMotor.get("rightVert");
        leftVerticalMotor = hardwareMap.dcMotor.get("leftVert");
        extension = hardwareMap.dcMotor.get("extension");

        //limit switches
        vertSwitch = hardwareMap.touchSensor.get("vertSwitch");//Todo: Changed cause screwed in wiring
        hortSwitch = hardwareMap.touchSensor.get("hortSwitch");

        //intake sensors
        intakeColorSensor = hardwareMap.get(ColorSensor.class, "colorChute");
        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "polu");

        //drivetrain
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        //set drivetrain directions
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        //set drivetrain behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftVerticalMotor.setDirection(DcMotor.Direction.REVERSE);
        rightVerticalMotor.setDirection(DcMotor.Direction.REVERSE);


        //set vert behaviors
        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //turn on color sensor light
        intakeColorSensor.enableLed(true);


        intakeDown();
        //hangersUp();
        intakeStopper.setPosition(0);

        currentSlideResolution = rightVerticalMotor.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()) {

            //RUNS DRIVE
            tankDrive();


            //CHECKS INTAKE SENSORS (experimental)
            //TODO: See if this works.
            /*
            getColor();

            boolean blockInColorSensors = false;
            //red block in the intake
            if(redValue > redThreshold){
                telemetry.addData("Block Color Detected: ", "Red: " + redValue);
                telemetry.update();

                blockInColorSensors = true;

                //checks if we want the color, gives feedback
                if (allianceIsBlue) {
                    //we don't want it
                    rumbleGamepads(1, 500);
                }
                else {
                    //we want it
                    rumbleGamepads(0.5, 200);
                }
            }

            //blue block in the intake
            else if(blueValue > blueThreshold){
                telemetry.addData("Block Color Detected: ", "Blue: " + blueValue);
                telemetry.update();

                blockInColorSensors = true;

                //checks if we want the color, gives feedback
                if (allianceIsBlue) {
                    //we want it
                    rumbleGamepads(0.5, 200);
                }
                else {
                    //we don't want it
                    rumbleGamepads(1, 500);
                }
            }

            //block at the distance sensor
            if (intakeDistanceSensor.getDistance(DistanceUnit.MM) < distanceThreshold) {
                telemetry.addData("Block Detected: ", intakeDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.update();

                //there are multiple blocks in the intake
                if (blockInColorSensors) {
                    rumbleGamepads(1, 500);
                }
            }

             */




            //HANG COMPONENTS

            //rack and pinion hangers
            if (gamepad1.x) {
                hangersUp();
            }
            else if (gamepad1.y) {
                hangersDown();
            }


            //INTAKE COMPONENTS

            //set intake stopper based on hort switch
            if (!hortSwitch.isPressed()) {
                intakeStopper.setPosition(0.105);
                Rumbled = false;
                gamepad2.stopRumble();
            }
            else if(hortSwitch.isPressed()) {
                intakeStopper.setPosition(0.46);
                if(!Rumbled){
                    gamepad2.rumble(500);
                    gamepad2.rumble(500);
                    Rumbled = true;
                }
            }

            //intake or outtake
            if (gamepad2.a && !blockFound) {
                intake.setPower(-1);
                intakeDown();
            }
            else if (gamepad2.b) {
                intake.setPower(1);
                blockFound = false;
                intakeRumbled = false;
                intakeUp();
            }
            else if(blockFound){
                intake.setPower(1);
                if(!intakeRumbled) {
                    gamepad2.rumble(500);
                    gamepad2.rumble(500);
                    //intakeRumbled = true;
                }


            }
            else {
                intake.setPower(0);
            }


            //CLAW AND DEPO COMPONENTS

            //open or close the claw
            if (gamepad2.y){
                openClaw();
            }
            else if(gamepad2.x){
                closeClaw();
            }


            //depo positions

            //grab spec
            if (gamepad2.dpad_left) {
                grabSpecPos();
            }

            //clip spec
            else if (gamepad2.dpad_right) {
                clipSpecPos();
            }

            //bucket drop
            else if (gamepad2.dpad_up) {
                basketPos();
            }

            //transfer reset
            else if (gamepad2.dpad_down) {
                transferPos();
            }








            //SLIDE COMPONENTS

            //vert up, down, or hold
            if (gamepad2.right_trigger > 0.3) {
                rightVerticalMotor.setPower(-1);
                leftVerticalMotor.setPower(1);
                specimenToHighBar = false;
            }
            else if (gamepad2.left_trigger > 0.3) {
                rightVerticalMotor.setPower(1);
                leftVerticalMotor.setPower(-1);
                specimenToHighBar = false;
            }
            else if(vertSwitch.isPressed() && !(gamepad2.right_trigger > 0.3) && !specimenToHighBar){
                rightVerticalMotor.setPower(0);
                leftVerticalMotor.setPower(0);

            }
           else if (specimenToHighBar) {
                leftVerticalMotor.setPower(1);
                rightVerticalMotor.setPower(-1);

                if (Math.abs(rightVerticalMotor.getCurrentPosition()) > currentSlideResolution+1750) {
                    rightVerticalMotor.setPower(-0.15);
                    leftVerticalMotor.setPower(0.15);
                    specimenToHighBar = false;
                }
            }
            else {
                rightVerticalMotor.setPower(-0.15);
                leftVerticalMotor.setPower(0.15);
            }


            //hort in, out, or neutral
            if (gamepad2.left_bumper) {
                extension.setPower(1);
                intakeTravelPos();
            }

            else if (gamepad2.right_bumper) {
                extension.setPower(-1);
                intakeTravelPos();
            }
            else {
                extension.setPower(0);
            }

            if(getColor("red") > redThreshold && redThreshold < 300){
                //blockFound = true;
            }
            //if(getColor("blue" > blueThreshold ))




            //UPDATE TELEMETRY
            colorTelmetry();
            slideTelemetry();
        }
    }


    private void intakeDown() {
        intakeTilt.setPosition(0.8);

    }

    private void intakeUp() {
        intakeTilt.setPosition(0.96);//0.61 or 0.71

    }

    private void intakeTravelPos() {
        intakeTilt.setPosition(0.83);

    }

    private void hangersUp() {
        leftHanger.setPosition(1);
        rightHanger.setPosition(0);
    }

    private void hangersDown() {
        leftHanger.setPosition(0.4);
        rightHanger.setPosition(0.6);
    }

    private void openClaw() {
        claw.setPosition(0.52);

    }

    private void closeClaw() {
        claw.setPosition(0.68);
        intakeStopper.setPosition(0.46);
    }

    private void grabSpecPos() {
        depoRight.setPosition(0.97);
        //depoLeft.setPosition(0.1);
        extendDepo.setPosition(0.56);
        wristClaw.setPosition(0.96);
    }

    private void clipSpecPos() {
       // depoLeft.setPosition(0.4);
        depoRight.setPosition(0.4);
        extendDepo.setPosition(0.4);
        wristClaw.setPosition(0.31);

        if(vertSwitch.isPressed()) {
            currentSlideResolution = rightVerticalMotor.getCurrentPosition();
        }
        specimenToHighBar = true;


    }

    private void transferPos() {
        depoRight.setPosition(0.33);
       // depoLeft.setPosition(0.91);
        wristClaw.setPosition(0.96); //0.8
        extendDepo.setPosition(0.51);//0.625
    }

    private void basketPos() {
        depoRight.setPosition(0.86);
      //  depoLeft.setPosition(0.3);
        extendDepo.setPosition(0.56);
        wristClaw.setPosition(0.96);
    }

    private double getColor(String color) {
        if(color.equals("red")){
            redValue = intakeColorSensor.red();
            return redValue;
        }
        if(color.equals("blue")){
            blueValue = intakeColorSensor.blue();
            return blueValue;
        }
        if(color.equals("green")){
            greenValue = intakeColorSensor.green();
            return greenValue;
        }
        if(color.equals("alpha")){
            alphaValue = intakeColorSensor.alpha();
            return alphaValue;
        }


        return 0;

    }

    private void colorTelmetry() {
        telemetry.addData("red", "%.2f", getColor("red"));
        telemetry.addData("blue", "%.2f", getColor("blue"));
        telemetry.addData("green", "%.2f", getColor("green"));
        telemetry.addData("alpha", "%.2f", getColor("alpha"));
        telemetry.addData("Slides:", Math.abs(rightVerticalMotor.getCurrentPosition()));
    }

    private void slideTelemetry() {
        if (vertSwitch.isPressed()) {
            telemetry.addData("Vert Sensor:", "Pressed");
            telemetry.update();


        } else {
            telemetry.addData("Vert Sensor:", "Not Pressed");
            telemetry.update();
        }

        if (hortSwitch.isPressed()) {
            telemetry.addData("Hort Sensor:", "Pressed");
            telemetry.update();

        } else {
            telemetry.addData("Hort Sensor:", "Not Pressed");
            telemetry.update();
        }


    }

    private void tankDrive() {

        //strafe right
        if (gamepad1.right_bumper) {
            frontLeft.setPower(1);
            backLeft.setPower(-1);
            frontRight.setPower(-1);
            backRight.setPower(1);
        }

        //strafe left
        else if (gamepad1.left_bumper) {
            frontLeft.setPower(-1);
            backLeft.setPower(1);
            frontRight.setPower(1);
            backRight.setPower(-1);

        }

        //creep forward
        else if (gamepad1.a) {
            frontLeft.setPower(0.4);
            frontRight.setPower(0.4);
            backLeft.setPower(0.4);
            backRight.setPower(0.4);
        }

        //creep backward
        else if (gamepad1.b) {
            frontLeft.setPower(-0.4);
            frontRight.setPower(-0.4);
            backLeft.setPower(-0.4);
            backRight.setPower(-0.4);
        }

        //normal drive
        else {
            frontLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.right_stick_y);
            backLeft.setPower(-gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.right_stick_y);
        }
    }

    private void rumbleGamepads(double intensity, int duration) {
        gamepad1.rumble(intensity, intensity, duration);
        gamepad2.rumble(intensity, intensity, duration);
    }
}