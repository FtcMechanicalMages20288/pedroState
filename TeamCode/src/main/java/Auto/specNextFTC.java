package Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import java.util.Set;

import Subsystems.Subsystems.Claw;
import Subsystems.Subsystems.Depo;
import Subsystems.Subsystems.Lift;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Specimen Auto NextFTC")
public class specNextFTC extends PedroOpMode {
    public specNextFTC() {
        super(Claw.INSTANCE, Lift.INSTANCE, Depo.INSTANCE);
    }
    private final Pose startPose = new Pose(9, 69, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(35, 69, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(10, 30, Math.toRadians(180));

    private final Pose lineUpControl = new Pose (49.8,47.2,Math.toRadians(180));
    private final Pose lineUp = new Pose(58,31,Math.toRadians(180));
    private final Pose firstPush = new Pose(22,31, Math.toRadians(180));
    private final Pose goBackControl = new Pose(63.5,31.3, Math.toRadians(180));
    private final Pose goBack = new Pose(58,23, Math.toRadians(180));
    private final Pose secondPush = new Pose(16, 19, Math.toRadians(180));
    private final Pose goBack2 = new Pose(20,19, Math.toRadians(180));
    private final Pose thirdPush = new Pose();
    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 125, Math.toRadians(90));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(40, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(63, 95, Math.toRadians(-90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(69, 112, Math.toRadians(-90));
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, pushChain;

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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        pushChain = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup1Pose), new Point(lineUpControl), new Point(lineUp)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), lineUp.getHeading())


                .addPath(new BezierLine(new Point(lineUp),new Point(firstPush)))
                .setLinearHeadingInterpolation(lineUp.getHeading(), firstPush.getHeading())


                .addPath(new BezierCurve(new Point(firstPush), new Point(goBackControl), new Point(goBack)) )
                .setLinearHeadingInterpolation(firstPush.getHeading(), goBack.getHeading())

                .addPath(new BezierLine(new Point(goBack),new Point(secondPush)))
                .setLinearHeadingInterpolation(goBack.getHeading(), secondPush.getHeading())

                .addPath(new BezierLine(new Point(secondPush),new Point(goBack2)))
                .setLinearHeadingInterpolation(secondPush.getHeading(), goBack2.getHeading())



                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPush), new Point(scorePose)))
                .setLinearHeadingInterpolation(secondPush.getHeading(), scorePose.getHeading())
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

    public Command initRoutine() {
        return new SequentialGroup(
                Claw.INSTANCE.close()
        );
    }
    public Command preLoadRoutine() {

        return new SequentialGroup(
               Depo.INSTANCE.resetDepo(),

        new ParallelGroup(
                       // new FollowPath(scorePreload),
                        Depo.INSTANCE.specDepo(),
                        Lift.INSTANCE.specPos()

                        //Depo.
                ),
                new SequentialGroup(
                        Lift.INSTANCE.depoSpec(),
                        Claw.INSTANCE.open()
                ),
                new Delay(1.0),

                new ParallelGroup(
                        Lift.INSTANCE.resetPos(),
                        Depo.INSTANCE.resetDepo()
                )
        );
    }

    public Command Cycle1Routine() {
        return new SequentialGroup(
                new FollowPath(grabPickup1),
                Claw.INSTANCE.close(),
                new ParallelGroup(
                        new FollowPath(scorePickup1),
                        Lift.INSTANCE.specPos(),
                        Depo.INSTANCE.specDepo()
                ),
                new ParallelGroup(
                        Claw.INSTANCE.open(),
                        Lift.INSTANCE.depoSpec()
                ),
                new Delay(1.0),
                Lift.INSTANCE.resetPos()
        );
    }

    public Command Cycle2Routine() {
        return new SequentialGroup(
                new FollowPath(scorePickup2),
                Claw.INSTANCE.close(),
                new ParallelGroup(
                        new FollowPath(scorePickup1),
                        Lift.INSTANCE.specPos(),
                        Depo.INSTANCE.specDepo()
                        //Depo.
                ),
                new ParallelGroup(
                        Claw.INSTANCE.open(),
                        Lift.INSTANCE.depoSpec()
                ),
                new Delay(1),
                Lift.INSTANCE.resetPos()
        );
    }

    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(9, 69, Math.toRadians(90)));
        buildPaths();
        initRoutine().invoke();
    }
    @Override
    public void onStartButtonPressed() {
        preLoadRoutine().invoke();
     //   Cycle1Routine().invoke();
    }

}