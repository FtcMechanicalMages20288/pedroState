package Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous (name = "autoPedro", group = "aGameAuto")
public class autoPedro extends OpMode {
    private Telemetry telemetryA;

    private Follower follower;

    private PathChain basketAuto;

    private boolean robotRanPath = false;


    /**
     * This initializes the Follower and creates the PathChain for the "BasketAuto". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */



      /*  telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly circular shape of radius "
                + ", starting on the right-most edge. So, make sure you have enough "
                + "space to the left, front, and back to run the OpMode.");
        telemetryA.update();*/


    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */


    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose (new Pose(24.435,112.448,Math.toRadians(-90)));

        basketAuto = follower.pathBuilder()
                .addPath(
                        //  Line 1
                        new BezierCurve(
                                new Point(24.435, 112.448, Point.CARTESIAN),
                                new Point(3.558, 82.794, Point.CARTESIAN),
                                new Point(23.960,82.557)
                        )).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .addPath(
                        //  Line 1
                        new BezierCurve(
                                new Point(23.960, 82.557, Point.CARTESIAN),
                                new Point(42.702, 69.509, Point.CARTESIAN),
                                new Point(23.723,47.684)
                        )).setTangentHeadingInterpolation()

               /* .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(12.888, 130.253, Point.CARTESIAN), new Point(21.995, 127.160, Point.CARTESIAN)
                        )
                )*/
                /* .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-20))
                 .addPath(
                         // Line 3
                         new BezierLine(
                                 new Point(21.995, 127.160, Point.CARTESIAN),
                                 new Point(12.888, 130.425, Point.CARTESIAN)
                         )
                 )
                 .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(-45))
                 .addPath(
                         // Line 4
                         new BezierLine(
                                 new Point(12.888, 130.425, Point.CARTESIAN),
                                 new Point(28.525, 131.284, Point.CARTESIAN)
                         )
                 )
                 .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                 .addPath(
                         // Line 5
                         new BezierLine(
                                 new Point(28.525, 131.284, Point.CARTESIAN),
                                 new Point(12.888, 130.425, Point.CARTESIAN)
                         )
                 )
                 .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                 .addPath(
                         // Line 6
                         new BezierLine(
                                 new Point(12.888, 130.425, Point.CARTESIAN),
                                 new Point(45.021, 129.737, Point.CARTESIAN)
                         )
                 )
                 .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                 .addPath(
                         // Line 7
                         new BezierLine(
                                 new Point(45.021, 129.737, Point.CARTESIAN),
                                 new Point(12.888, 130.597, Point.CARTESIAN)
                         )
                 )*/.build();

        follower.followPath(basketAuto);
    }

    @Override
    public void loop() {
        follower.update();
       if(!robotRanPath) {
           if (follower.atParametricEnd()) {
               follower.followPath(basketAuto, true);
               robotRanPath = true;
           }
       }
       else{

       }
    }
}

