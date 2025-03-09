package pedroPathing.Subsystems;


import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToSeperatePositions;

import java.util.Map;

public class Depo extends Subsystem {
    // BOILERPLATE
    public static final Depo INSTANCE = new Depo();
    private Depo() { }

    // USER CODE
    public Servo depoRight, depoLeft, wristClaw;
    public String wrist = "wristClaw";
    public String right  = "RightDepo";
    public String left= "LeftDepo";


    public Command resetDepo() {
        Double rotate = 0.6;
        Double rightTilt = 0.03;
        Double leftTilt = 0.97;

        return new MultipleServosToSeperatePositions(
                Map.of(
                        depoRight, rightTilt,
                        depoLeft, leftTilt,
                        wristClaw, rotate
                ),
                this);
    }

    public Command depositSamp() {
        Double rotate = 0.6;
        Double rightTilt = 0.63;
        Double leftTilt = 0.37;

        return new MultipleServosToSeperatePositions(
                Map.of(
                        depoRight, rightTilt,
                        depoLeft, leftTilt,
                        wristClaw, rotate
                ),

                this);
    }

    public Command specPic() {
        Double rotate = 0.6;
        Double rightTilt = 0.67;
        Double leftTilt = 0.33;

        return new MultipleServosToSeperatePositions(
                Map.of(
                        depoRight, rightTilt,
                        depoLeft, leftTilt,
                        wristClaw, rotate
                ),

                this);
    }

    public Command specDepo() {
        Double rotate = 0.6;
        Double rightTilt = 0.9;
        Double leftTilt = 0.1;

        return new MultipleServosToSeperatePositions(
                Map.of(
                        depoRight, rightTilt,
                        depoLeft, leftTilt,
                        wristClaw, rotate
                ),

                this);
    }

    @Override
    public void initialize() {
        wristClaw = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, wrist);
        depoRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, right);
        depoLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, left);

    }
}