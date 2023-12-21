package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Stands for <b>Operator Interface</b>
 */
public class OI 
{
        public final XboxController xboxController = new XboxController(frc.robot.Constants.OI.kXboxControllerPort);
        public final JoystickButton A = new JoystickButton(xboxController, XboxController.Button.kA.value);
        public final JoystickButton B = new JoystickButton(xboxController, XboxController.Button.kB.value);
        public final JoystickButton X = new JoystickButton(xboxController, XboxController.Button.kX.value);
        public final JoystickButton Y = new JoystickButton(xboxController, XboxController.Button.kY.value);
        public final POVButton DPadUP = new POVButton(xboxController, 0);
        public final POVButton DPadDOWN = new POVButton(xboxController, 180);
        public final POVButton DpadRIGHT = new POVButton(xboxController,90);
        public final POVButton DPadLEFT = new POVButton(xboxController, 270);
        public final JoystickButton LeftBumper = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
        public final JoystickButton RightBumper = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);

}
