import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robocol.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
/*
@Disabled
@Autonomous(name = "NextFTC Autonomous Program 2 Java")
public class nextFTC_Example extends PedroOpMode {
    public nextFTC_Example() {
        super(Claw.INSTANCE, Lift.INSTANCE);
    }

    private final Pose startPose = new Pose(9.0, 60.0, Math.toRadians(0.0));
    private final Pose finishPose = new Pose(37.0, 50.0, Math.toRadians(180.0));

    private PathChain move;

    public void buildPaths() {
        move = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(finishPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading())
                .build();
    }

    public Command secondRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(move),
                        Lift.INSTANCE.toHigh()
                ),
                new ParallelGroup(
                        Claw.INSTANCE.open(),
                        Lift.INSTANCE.toMiddle()
                ),
                new Delay(1.0),
                Lift.INSTANCE.toLow()
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().acknowledge(); //(or .invoke)
    }
}*/