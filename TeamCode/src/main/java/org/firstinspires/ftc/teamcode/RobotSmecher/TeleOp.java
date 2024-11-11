package org.firstinspires.ftc.teamcode.RobotSmecher;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotSmecher.Subsystems.IntakeSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Main TeleOp")
public class TeleOp extends CommandOpMode {

    private DriveSubsystem drive;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        GamepadEx driver = new GamepadEx(gamepad1);
        drive.setAxes(driver::getLeftY, driver::getLeftX, driver::getRightX);

        register(drive, arm, intake, outtake);

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(arm::toggleArmState);

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> intake.changeState(IntakeSubsystem.State.INTAKE));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> intake.changeState(IntakeSubsystem.State.OUTTAKE));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(outtake::toggleOuttakeState);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(outtake::lowerOuttake);
    }

    @Override
    public void run() {

    }
}
