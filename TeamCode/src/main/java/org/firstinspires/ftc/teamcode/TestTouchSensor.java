package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Test OpMode for a REV Touch Sensor (or Magnetic Limit Switch).
 *
 * Hardware config name: "touch_sensor"
 *   - REV Touch Sensor must be on a digital port (1, 3, 5, or 7)
 *   - REV Magnetic Limit Switch can be on any digital port
 *
 * Displays on Driver Station:
 *   - Current pressed/released state
 *   - Total press count
 *   - Duration of the current press (while held)
 *   - Duration of the last completed press
 */
@TeleOp(name = "Test: Touch Sensor", group = "Test")
public class TestTouchSensor extends LinearOpMode {

    private static final String SENSOR_NAME = "touch_sensor";

    @Override
    public void runOpMode() {

        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, SENSOR_NAME);

        ElapsedTime pressTimer = new ElapsedTime();
        int pressCount = 0;
        boolean wasPressed = false;
        double lastPressDuration = 0;

        telemetry.addData("Status", "Ready — press START");
        telemetry.addData("Sensor", SENSOR_NAME);
        telemetry.update();

        waitForStart();

        pressTimer.reset();

        while (opModeIsActive()) {
            boolean isPressed = touchSensor.isPressed();

            // Detect press/release transitions
            if (isPressed && !wasPressed) {
                // Rising edge — sensor just pressed
                pressCount++;
                pressTimer.reset();
            } else if (!isPressed && wasPressed) {
                // Falling edge — sensor just released
                lastPressDuration = pressTimer.seconds();
            }

            // Telemetry
            telemetry.addData("State", isPressed ? "PRESSED" : "released");
            telemetry.addData("Press count", pressCount);

            if (isPressed) {
                telemetry.addData("Hold duration (s)", "%.2f", pressTimer.seconds());
            } else {
                telemetry.addData("Last press duration (s)",
                        pressCount > 0 ? String.format("%.2f", lastPressDuration) : "n/a");
            }

            telemetry.update();

            wasPressed = isPressed;
        }
    }
}
