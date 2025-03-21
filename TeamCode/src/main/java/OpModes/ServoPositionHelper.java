/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package OpModes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


/*
 * This file contains example code designed to help your team find the signal to send
 * your servos so that they move to your desired location.
 * It is a TeleOp mode, and uses the gamepad to increment or decrement the servo position
 * while displaying that position as Telemetry on your Driver Station.
 *
 * To use this program, start with the servo mounted to your robot and the servo attachment
 * not installed yet. Then hit "init" on your drivers station to move the servo to a known "halfway"
 * position. This should be a desired state of your mechanism, like a wrist pointed straight out
 * ready to interact with game elements. On init, this code moves the servo to the "0.5" position.
 * This is halfway through the throw of the servo. So if your servo has 180° of range, this starts
 * it at 90°.
 *
 * Once your servo attachment is installed, hit the START button. This will allow you to drive the
 * servo using your gamepad. Pressing the Y(△) button will increase the signal, while pressing
 * A(X) will decrease the position.
 * If the servo is incrementing too far, or not far enough. You can adjust the rate by pressing
 * D-pad up, or d-pad down.
 */

@TeleOp(name="Servo Position Helper", group="Concept")
public class ServoPositionHelper extends LinearOpMode {

    // Declare OpMode member.
    private Servo servo, servo1 = null;

    /*
     * Create a variable which we will modify with our code. Eventually we will instruct
     * the servo to run to the position captured by this variable.
     */
    private double servoPosition = 0.5;

    // Create a variable for size of each "step" that we will increment or decrement our servo position by.
    private double positionAdjustment = 0.05;

    // This variable captures how much we need to increment or decrement the step size by
    private final double STEP_ADJUSTMENT = 0.01;

    /*
     * This variable is the maximum position we want to send to the servo.
     * Some servos do not operate well went sent a signal too large, or too small.
     * Most Hitec Linear servos for example only respond to signals within a 1050-1950µsec range.
     * Converted to 0-1, that means we should not send a Hitec Linear Servo less than 0.25, or more than 0.75.
     */
    private final double MIN_POSITION = 0;
    private final double MAX_POSITION = 1;

    // These booleans are used in the "rising edge detection"
    private boolean previousGamepadY = false;
    private boolean previousGamePadA = false;
    private boolean previousGamePadUp = false;
    private boolean previousGamePadDown = false;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "RightDepo");
        servo.setDirection(Servo.Direction.REVERSE);
        //servo1 = hardwareMap.get(Servo.class, "RightDepo");
        //servo1.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");

        //servo1.setPosition(.98);
        servo.setPosition(0.5);
        /*
         * Initialize the hardware variables, string here must exactly match the name of a configured
         * servo in the Robot Configuration on your Driver Station.
         */


        /*
         * Set the servo to an initial position of 0.5, we do this before the while (opModeIsActive())
         * loop starts so that we can install the servo attachment in a "known" position.
         * There isn't anything special about doing this at 0.5. We could instead choose 0, or 1!
         */
        //servoPosition = 1;
        //servo.setPosition(servoPosition);
       // servo.setDirection(Servo.Direction.REVERSE);

       // servo1.setPosition(servoPosition-0.02);



        telemetry.addData("Depo Left Position: ",servo.getPosition());
      //  telemetry.addData("Depo Right Position: ",servo1.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
             * when the user presses a button. We have a few ways too design how our code responds.
             * The simplest is to preform an action whenever the button is pressed, in practice,
             * this means that for every cycle (fraction of a second) that the button is held, it
             * will run the code in your if statement.
             * Here we want the code in our statement to only execute once every time that the button
             * is pressed. To do this, we need to implement "rising edge detection" this works by
             * checking both the current state of the button (which we store in a boolean called
             * currentGamepadY) and the state of the button in the previous loop (which we store in
             * previousGamepadY). If previousGamepadY is false and currentGamepadY is true, then
             * the button went from not being pressed, to being pressed. Once it is held down, both
             * booleans will be true. So we can ignore those and just look for the change from one
             * cycle to the next.
             */
            boolean currentGamepadY = gamepad1.y;
            boolean currentGamepadA = gamepad1.a;
            boolean currentGamepadUp = gamepad1.dpad_up;
            boolean currentGamepadDown = gamepad1.dpad_down;

            // Check to see if the user is clicking the Y(△) button on the gamepad.
            if (currentGamepadY && !previousGamepadY){
                // += is an operator that lets us add the step variable without overwriting the servoPosition variable.
                servoPosition += positionAdjustment;
            } else if (currentGamepadA && !previousGamePadA){
                // We use an else if statement here so that we only check if A(x) is pressed after we know
                // that the Y(△) button is not pressed.
                servoPosition -= positionAdjustment;
            }

            // Here we modify the step size if the user clicks D-pad up or D-pad down.
            if (currentGamepadUp && !previousGamePadUp){
                positionAdjustment += STEP_ADJUSTMENT;
            } else if (currentGamepadDown && !previousGamePadDown){
                positionAdjustment -= STEP_ADJUSTMENT;
            }

            // Check to see if we're setting the servoPosition to less than the min, or more than the max.
            if (servoPosition > MAX_POSITION){
                servoPosition = MAX_POSITION;
            } else if (servoPosition < MIN_POSITION){
                servoPosition = MIN_POSITION;
            }

            /*
             * Finally, set the servo to the servoPosition variable. We do this only once per loop
             * so that we can be sure not to write conflicting positions to the servo.
             */
            servo.setPosition(servoPosition);
            //servo1.setPosition(servoPosition);

            // Because our logic has finished, we set our "previousGamepad" booleans to the current ones.
            previousGamepadY = currentGamepadY;
            previousGamePadA = currentGamepadA;
            previousGamePadUp = currentGamepadUp;
            previousGamePadDown = currentGamepadDown;

            // Show the servo position
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Servo Step Size", positionAdjustment);
            telemetry.update();
        }
    }
}