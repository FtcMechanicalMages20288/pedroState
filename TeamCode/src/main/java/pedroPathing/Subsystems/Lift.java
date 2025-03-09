package pedroPathing.Subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
public class Lift extends Subsystem {
    // BOILERPLATE
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    // USER CODE

    public MotorEx leftVert;
    public MotorEx rightVert;

    public MotorGroup Verts;

    public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));

    public Command specPos() {
        return new RunToPosition(Verts, // MOTOR TO MOVE
                925, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command depoSpec() {
        return new RunToPosition(Verts, // MOTOR TO MOVE
                700, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command resetPos() {
        return new RunToPosition(Verts, // MOTOR TO MOVE
                0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command depoSampPos() {
        return new RunToPosition(Verts, // MOTOR TO MOVE
                2250, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        leftVert = new MotorEx("leftVert");
        rightVert = new MotorEx("rightVert");
        Verts = new MotorGroup(leftVert, rightVert);
    }
}