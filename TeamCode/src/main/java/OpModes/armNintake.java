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


@TeleOp(name = "arm Intake test")

public class armNintake extends LinearOpMode {

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
    private double redThreshold = 0;
    private double blueThreshold = 0;
    private double distanceThreshold = 40;
    private int currentSlideResolution;

    private boolean allianceIsBlue = true;
    private boolean specimenToHighBar = false;
    private boolean Rumbled = false;

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

        //set vert behaviors
        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //turn on color sensor light
        waitForStart();
        intakeTilt.setPosition(0.8);
        depoLeft.setPosition(0);
        while (opModeIsActive()) {

            if(gamepad1.a){
                depoLeft.setPosition(0);
                depoRight.setPosition(0);
              //  transferPos();
            }
            else if (gamepad1.b){
                depoLeft.setPosition(1);
                depoRight.setPosition(1);
                //clipSpecPos();
            }
            else if(gamepad1.x){
                grabSpecPos();
            }
            else if(gamepad1.y){
                basketPos();
            }

            if(gamepad1.right_bumper){
                openClaw();
            }
            else if(gamepad1.left_bumper){
                closeClaw();
            }
            colorTelmetry();
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
        claw.setPosition(0.48);

    }

    private void closeClaw() {
        claw.setPosition(0.59);
        intakeStopper.setPosition(0.46);
    }

    private void grabSpecPos() {
        //depoRight.setPosition(1);
        depoLeft.setPosition(0.94);
        extendDepo.setPosition(0.67);
        wristClaw.setPosition(0.96);
    }

    private void clipSpecPos() {
        depoLeft.setPosition(0.34);
        //depoRight.setPosition(0.1);
        extendDepo.setPosition(0.6);
        wristClaw.setPosition(0.31);

        if(vertSwitch.isPressed()) {
            currentSlideResolution = rightVerticalMotor.getCurrentPosition();
        }
        specimenToHighBar = true;


    }

    private void transferPos() {
        // depoRight.setPosition(0.03);
        depoLeft.setPosition(0.25);
        wristClaw.setPosition(0.96); //0.8
        extendDepo.setPosition(0.58);//0.625
    }

    private void basketPos() {
        //depoRight.setPosition(0.63);
        depoLeft.setPosition(0.8);
        extendDepo.setPosition(0.67);
        wristClaw.setPosition(.96);
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
        telemetry.update();
    }
}