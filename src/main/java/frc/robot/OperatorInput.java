// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OperatorInput {
    XboxController movementController;
    XboxController armController;

    public OperatorInput() {
        movementController = new XboxController(0);
        armController = new XboxController(1);
    }

    public double getLeftXMovement() {
        return movementController.getLeftX();
    }
    public double getLeftYMovement() {
        return movementController.getLeftY();
    }
    //Turn
    public double getRightXMovement() {
    
        return movementController.getRightX();
    }
    // run Intake motors
    public double getRightTriggerAxisArm() {
        return armController.getRightTriggerAxis();
    }
    //Outtake motors
    public double getLeftTriggerAxisArm() {
        return armController.getLeftTriggerAxis();
    }
    //Toggles intake pistons
    public boolean getRightBumperPressedArm() {
        return armController.getRightBumperPressed();
    }
    //Used for claw toggle
    public boolean getBPressedArm() {
        return armController.getBButtonPressed();
    }
    //Extends and reduces telescope
    public double getLeftYArm() {
        return armController.getLeftY();
    }
    //Pivots arm
    public double getRightYArm() {
        return armController.getRightY();
    }
    //https://pdocs.kauailabs.com/navx-mxp/examples/field-oriented-drive/
    //Eg of field oriented. If robot is rotate 180 degrees and left joystick Y is held forwards,
    //The robot will move forward relative to the field, not its own frame of reference
    //IE. towards the opponents side, and not towards the driver
    public boolean getBackPressedMovement() {
        return movementController.getBackButtonPressed();
    }

    public boolean configureButtonBindings() {
        return movementController.getStartButtonPressed();
    }

    


}
