package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.commands.State.REST;
import static org.firstinspires.ftc.teamcode.commands.State.START;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.State;


/*  POSITIONS:



*/

public class IntakeEx {
    private static int pivotPos = 0;
    private DcMotorEx intakeMotor;
    private Arm arm;
    private Wrist wrist;
    private Claw claw;
    private Telemetry telemetry;
    private ElapsedTime intakeTimer;
    private GamepadEx driver, operator;
    private boolean doClose = true, doClamp = true;
    private State lastState = State.REST;

    public IntakeEx(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx drive, GamepadEx op)  {
        this.telemetry = telemetry;
        driver = drive;
        operator = op;
        intakeTimer = new ElapsedTime();

        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0.0);

        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    public void executeTele(State state) {
        if(!(lastState==state)) {
            intakeTimer.reset();
            lastState = state;
        }
        int motorPow = 0;
        switch (state) {
            case INTAKE:
                if(intakeMotor.getCurrentPosition() < 900) { //intake is out
                    telemetry.addData("reached here: ", true);
                    motorPow = 1;
                    telemetry.addData("reached here: ", motorPow);
                    intakeMotor.setPower(motorPow);
                } else {
                    motorPow = 0;
                    telemetry.addData("reached here: ", motorPow);
                    intakeMotor.setPower(motorPow);
                    arm.intake();
                    wrist.intake();
                    claw.openRight();
                    claw.openLeft();
                    doClose = true;
                }
                break;

            case REST:
                if(intakeMotor.getCurrentPosition() > 900) {
                    if (doClose) {
                        claw.closeRight();
                        claw.closeLeft();
                    } else {
                        claw.openRight();
                        claw.openLeft();
                    }
                    if(intakeTimer.seconds()>0.5) {
                        intakeManualIn();
                    }
                }
                else { //intake is out
                    arm.transition();
                    wrist.transition();
                    claw.closeRight();
                    claw.closeLeft();
                    if(intakeMotor.getCurrentPosition() > 10) { //intake is all the way in
                        intakeManualIn();
                    } else {
                        holdIntake();
                    }
                }
                break;

            case DEPOSIT:

                break;

            case SHORT_DEPOSIT:
                if(intakeMotor.getCurrentPosition() < 900) { //intake is out
                    intakeManualOut();
                } else {
                    holdIntake();
                    arm.shortDeposit();
                    wrist.shortDeposit();
                    doClose = false;
                }
                break;
        }

        telemetry.addData("Intake timer: ", intakeTimer.seconds());
        telemetry.addData("Intake slide pos: ", intakeMotor.getCurrentPosition());
        telemetry.addData("Current state: ", state);
    }

    //START, INTAKE, REST, SHORT_DEPOSIT, EXTEND, DEPOSIT, RETRACT, RETURNING, CLIMB, LAUNCH_DRONE

    public void intakeManualOut() {
        intakeMotor.setPower(1.0);
    }

    public void intakeManualIn() {
        intakeMotor.setPower(-1.0);
    }

    public void holdIntake() {
        intakeMotor.setPower(0.0);
    }

    public DcMotorEx getIntake() {
        return intakeMotor;
    }
}
