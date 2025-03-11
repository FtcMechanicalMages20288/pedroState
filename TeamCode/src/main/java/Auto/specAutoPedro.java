package Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Specimen Auto Pedro", group = "Examples")
public class specAutoPedro extends OpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight, extension;
    private DcMotor intake, rightVerticalMotor, leftVerticalMotor;
    private Servo extendDepo, depoLeft, depoRight, claw, leftHanger, rightHanger, holdChute, intakeTilt, wristClaw;
    private TouchSensor vertSwitch, hortSwitch;
    private ColorSensor colorChute;



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 69, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(33.5, 69, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(10, 30, Math.toRadians(180));

    private final Pose lineUpControl = new Pose (19,34,Math.toRadians(180));
    private final Pose lineUp = new Pose(58,32,Math.toRadians(180));
    private final Pose firstPush = new Pose(22,29, Math.toRadians(180));
    private final Pose goBackControl = new Pose(64,31.3, Math.toRadians(180));
    private final Pose goBack = new Pose(58,23, Math.toRadians(180));
    private final Pose secondPush = new Pose(20, 21, Math.toRadians(180));

    private final Pose goBack2Control = new Pose(63.19464787788005,25.53755903031544, Math.toRadians(180));
    private final Pose goBack2 = new Pose(58,12, Math.toRadians(180));
    private final Pose thirdPush = new Pose(20, 12, Math.toRadians(180));

    private final Pose goBack3 = new Pose(13,12, Math.toRadians(180));


    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 125, Math.toRadians(90));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(40, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(63, 95, Math.toRadians(-90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(69, 112, Math.toRadians(-90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path  park;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, pushChain;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        // Scored preloaded block
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))

              .addParametricCallback(0.1, () -> slidesRunUP(1150))
                .addParametricCallback(0.1, this::specimenClip)

                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        pushChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(lineUpControl), new Point(lineUp)))
                .addParametricCallback(0.05, this::depoReset)
                .addParametricCallback(0.6, this::depoStop)
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp.getHeading())


                .addPath(new BezierLine(new Point(lineUp),new Point(firstPush)))
                .setLinearHeadingInterpolation(lineUp.getHeading(), firstPush.getHeading())


                .addPath(new BezierCurve(new Point(firstPush), new Point(goBackControl), new Point(goBack)) )
                .setLinearHeadingInterpolation(firstPush.getHeading(), goBack.getHeading())

                .addPath(new BezierLine(new Point(goBack),new Point(secondPush)))
                .setLinearHeadingInterpolation(goBack.getHeading(), secondPush.getHeading())

                .addPath(new BezierCurve(new Point(secondPush), new Point(goBack2Control), new Point(goBack2)))
                .setLinearHeadingInterpolation(secondPush.getHeading(), goBack2.getHeading())
                .addParametricCallback(0.1, ()-> slidesRunUP(125))

                .addPath(new BezierLine(new Point(goBack2),new Point(thirdPush)))
                .addParametricCallback(0.1, this::pickupSpecimen)
                .setLinearHeadingInterpolation(goBack2.getHeading(), thirdPush.getHeading())

                .addPath(new BezierLine(new Point(thirdPush),new Point(goBack3)))

                .setLinearHeadingInterpolation(thirdPush.getHeading(), goBack3.getHeading())


                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goBack3), new Point(scorePose)))
                .setLinearHeadingInterpolation(goBack3.getHeading(), scorePose.getHeading())
                .build();



        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierLine(new Point(scorePose), new Point(pickup1Pose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
               setPathState(21);
                break;

                //Pickup1 is skipped until the pushChain

            case 21:

                if(!follower.isBusy()){
                    slidesRunDown(-100);//function resets the tick count every time cause otherwise its cooked, idk y \ ALSO try a lower -negative number if its going too far lol
                    telemetry.addData("Slides: ", "Worked");
                    telemetry.update();

                }

                if(!rightVerticalMotor.isBusy() && !follower.isBusy()){
                    openClaw();
                    setPathState(2);
                }
                break;

              /* if(!rightVerticalMotor.isBusy() && !follower.isBusy()){
                    openClaw();
                }*/


            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;

            case 2:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushChain,true);
                    setPathState(3); //3
                }
                break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    closeClaw();

                    specimenClip();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    //follower.followPath(scorePickup1,true);
                    //setPathState(4);
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        initHw();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        closeClaw();


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void slidesRunUP(int pos){
        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftVerticalMotor.setTargetPosition(pos);
        rightVerticalMotor.setTargetPosition(pos);


            leftVerticalMotor.setPower(1);
            rightVerticalMotor.setPower(1);


        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);





    }
    public void slidesRunDown(int pos){
        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftVerticalMotor.setTargetPosition(pos);
        rightVerticalMotor.setTargetPosition(pos);


        leftVerticalMotor.setPower(-1);
        rightVerticalMotor.setPower(-1);


        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




    }



    public void depoReset(){
        depoLeft.setPosition(0.97);
        depoRight.setPosition(0.03);
        extendDepo.setPosition(0.71);
        wristClaw.setPosition(0.6);

        if(!hortSwitch.isPressed()) {
            leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftVerticalMotor.setPower(-1);
            rightVerticalMotor.setPower(-1);
        }
      else{
          leftVerticalMotor.setPower(0);
          rightVerticalMotor.setPower(0);

        }


    }

    public void depoStop(){
        leftVerticalMotor.setPower(0);
        rightVerticalMotor.setPower(0);
        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Slides:", "Stopped" );
        telemetry.update();
    }

    public void pickupSpecimen(){
        depoLeft.setPosition(0.97);
        depoRight.setPosition(0.03);
        extendDepo.setPosition(0.4);
        wristClaw.setPosition(0.6);

    }


    public void specimenClip(){
        depoLeft.setPosition(0.37);
        depoRight.setPosition(0.63);
        extendDepo.setPosition(.67);
        wristClaw.setPosition(0.6);
    }

    public void closeClaw(){
        claw.setPosition(0.58);

    }

    public void openClaw() {
        claw.setPosition(0.48);
    }

    public void initHw(){
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
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightVerticalMotor.setDirection(DcMotor.Direction.REVERSE);

    }


}

