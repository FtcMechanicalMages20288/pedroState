package Subsystems.Subsystems;


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
    public Servo depoRight, depoLeft, wristClaw,extendArm;
    public String wrist = "wristClaw";
    public String right  = "RightDepo";
    public String left= "LeftDepo";
    public String extend = "extendDepo";


    public Command resetDepo() {
        Double rotate = 0.6;
        Double rightTilt = 0.03;
        Double leftTilt = 0.97;
        Double extendDepo = 0.655 ;


        return new MultipleServosToSeperatePositions(
                Map.of(
                        depoRight, rightTilt,
                        depoLeft, leftTilt,
                        wristClaw, rotate,
                        extendArm, extendDepo
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
        Double extendDepo = 0.71;

        return new MultipleServosToSeperatePositions(
                Map.of(
                        depoRight, rightTilt,
                        depoLeft, leftTilt,
                        wristClaw, rotate,
                        extendArm, extendDepo
                        ),

                this);
    }

    public Command specDepo() {
        Double rotate = 0.6;
        Double rightTilt = 0.63;
        Double leftTilt = 0.37;
        Double extendDepo = .67;
        return new MultipleServosToSeperatePositions(
                Map.of(
                        depoRight, rightTilt,
                        depoLeft, leftTilt,
                        wristClaw, rotate,
                        extendArm, extendDepo
                        ),

                this);
    }

    @Override
    public void initialize() {
        wristClaw = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, wrist);
        depoRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, right);
        depoLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, left);
        extendArm = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, extend);

    }
}