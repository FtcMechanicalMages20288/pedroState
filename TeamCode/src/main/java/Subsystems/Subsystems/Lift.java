package Subsystems.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.SingleFunctionCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.ResetEncoder;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
public class Lift extends Subsystem {
    // BOILERPLATE
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    // USER CODE

    public MotorEx leftVert;
    public MotorEx rightVert;

    public MotorGroup Verts;

    public PIDFController controller = new PIDFController(0.018, 0.0, 0.001, new StaticFeedforward(0.0));


    public Command specPos() {

        return new RunToPosition(Verts, // MOTOR TO MOVE
                1200, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM

      //  return new HoldPosition(Verts, controller,this);
    }


    public Command holdSlides(){
        return new HoldPosition(Verts, controller,this);
    }


    public Command depoSpec() {
        return new RunToPosition(Verts, // MOTOR TO MOVE
                1400, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM


    }

    public Command resetPos() {

        return new RunToPosition(Verts, // MOTOR TO MOVE
                0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public class resetVerts extends SingleFunctionCommand{
        @Override
        public boolean run(){
            rightVert.resetEncoder();
            leftVert.resetEncoder();
            return false;
        }
    }



    public Command resetRightSlide(){
        return new ResetEncoder(rightVert, this);
    }
    public Command resetLeftSlide(){
        return new ResetEncoder(leftVert, this);
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
        rightVert.setDirection(DcMotor.Direction.REVERSE);


        Verts = new MotorGroup(leftVert, rightVert);
    }
}