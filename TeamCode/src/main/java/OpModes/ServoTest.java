package OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {

    private Servo theOneServo,  theTwoServo;

    @Override
    public void runOpMode() throws InterruptedException {

        theOneServo = hardwareMap.servo.get("wrist");


        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                theOneServo.setPosition(1);
            }
            else if (gamepad1.b) {
                theOneServo.setPosition(0);
            }

        }
    }
}
