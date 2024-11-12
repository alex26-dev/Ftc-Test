package org.firstinspires.ftc.teamcode.RobotSmecher;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.OuttakeSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Main TeleOp")
public class TeleOp extends CommandOpMode {

    private DriveSubsystem drive;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private OuttakeSubsystem outtake;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap, elevator);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);
        drive.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        arm.setBoolSupplier(intake::isSampleCollected);
        outtake.setBooleanSupplier(intake::isSampleCollected, arm::isArmRetracted);

        register(drive, arm, intake, elevator, outtake);

        // Brakes
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(() -> drive.setPowerLimit(0.5)).whenReleased(() -> drive.setPowerLimit(1.0));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(() -> drive.setPowerLimit(0.33)).whenReleased(() -> drive.setPowerLimit(1.0));

        // Intake
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(arm::toggleArmState); // Moves intake forward in order to reach samples
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(intake::toggleIntake); // Picks up sample if no sample is detected or spits it out if it is detected

        // Outtake
        // Preset elevator positions
        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> elevator.changeElevatorState(ElevatorSubsystem.ElevatorState.HIGH_CHAMBER));
        driver2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> elevator.changeElevatorState(ElevatorSubsystem.ElevatorState.HIGH_BASKET));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> elevator.changeElevatorState(ElevatorSubsystem.ElevatorState.BOTTOM)); // Lowers the elevator as much as possible
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> elevator.changeElevatorState(ElevatorSubsystem.ElevatorState.TOP)); // Raises the elevator as much as possible
        // Moves the elevator as much as the driver wants
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(elevator::raiseElevator);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(elevator::lowerElevator);

        // Action claw
        driver2.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(outtake::toggleClawState);

        // Use in endgame for clinging onto low rung
        // Elevator goes from lowered -> raised above the low rung -> goes down to cling on to bar
         driver2.getGamepadButton(GamepadKeys.Button.START).whenPressed(elevator::toggleHangState);
    }

    @Override
    public void run() {

    }
}
