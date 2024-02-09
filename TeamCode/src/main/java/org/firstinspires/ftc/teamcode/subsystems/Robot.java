package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.commands.State.*;

//import org.firstinspires.ftc.robotcore.external.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.opmodes.teleop.BaseOp;

public class Robot {

    public Drivetrain drivetrain;
    private static ElapsedTime robotTimer = new ElapsedTime();
    public boolean doClose = true, doClamp = false, purpleDeosited = false;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public Deposit deposit;
    public DcMotorEx leftSlide, rightSlide;
    public Intake intake;
    private State state = State.START;
    private GamepadEx driver = null;
    private GamepadEx operator = null;
    public Launcher plane;
    public State lastState = START;
    Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        drivetrain = new Drivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        leftSlide = (DcMotorEx) hardwareMap.dcMotor.get("leftSlide");
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setPower(0.0);
        rightSlide = (DcMotorEx) hardwareMap.dcMotor.get("rightSlide");
        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setPower(0.0);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        plane = new Launcher(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx drive, GamepadEx op){
        this.telemetry = telemetry;

        drivetrain = new Drivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        leftSlide = (DcMotorEx) hardwareMap.dcMotor.get("leftSlide");
        rightSlide = (DcMotorEx) hardwareMap.dcMotor.get("rightSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotorEx.Direction.REVERSE);

        //rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        driver = drive;
        operator = op;
        plane = new Launcher(hardwareMap);

        //arm = new Arm(hardwareMap);
        //claw = new Claw(hardwareMap);
        //slides = new LinearSlides(hardwareMap);
        //plane = new Launcher(hardwareMap);
    }


    public void executeTele() {
        driver.readButtons();
        operator.readButtons();
        telemetry.addData("Time: ", robotTimer.seconds());
        telemetry.addData("B is down: ", operator.isDown(GamepadKeys.Button.B));
        if(!(lastState==state)) {
            robotTimer.reset();
            lastState = state;
        }
        switch(state) {
            case START:
                break;
            case REST:
                leftSlide.setPower(0.0);
                rightSlide.setPower(0.0);
                if(!doClamp) {
                    deposit.openLeftClamp();
                    deposit.openRightClamp();
                } else {
                    deposit.closeLeftClamp();
                    deposit.closeRightClamp();
                }
                if(intake.getIntake().getCurrentPosition() < -900) {
                    if (doClose) {
                        claw.closeRight();
                        claw.closeLeft();
                    } else {
                        claw.openRight();
                        claw.openLeft();
                    }
                    if(robotTimer.seconds()>0.5) {
                        intake.intakeManualIn();
                    }
                }
                else { //intake is out
                    arm.transition();
                    wrist.transition();
                    claw.closeRight();
                    claw.closeLeft();
                    if(intake.getIntake().getCurrentPosition() < -40) { //intake is all the way in
                        intake.intakeManualIn();
                    } else {
                        intake.holdIntake();
                    }
                }
                break;

            case INTAKE:
                doClamp = false;
                if(intake.getIntake().getCurrentPosition() > -900) { //intake is out
                    intake.intakeManualOut();
                } else {
                    intake.holdIntake();
                    arm.intake();
                    wrist.intake();
                    claw.openRight();
                    claw.openLeft();
                    doClose = true;
                }
                break;

            case SHORT_DEPOSIT:
                doClamp = false;
                if(intake.getIntake().getCurrentPosition() > -900) { //intake is out
                    intake.intakeManualOut();
                } else {
                    intake.holdIntake();
                    arm.shortDeposit();
                    wrist.shortDeposit();
                    doClose = false;
                }
                break;

            case EXTEND:
                doClamp=true;
                doClose=false;
                claw.looseLeft();
                claw.looseRight();
                if(leftSlide.getCurrentPosition()<200) {
                    if ( robotTimer.seconds() < 0.2) {
                        deposit.readyToClamp();
                    }
                    if(doClamp) {
                        if (robotTimer.seconds() > 0.2 && robotTimer.seconds() < 0.5) {
                            deposit.closeLeftClamp();
                            deposit.closeRightClamp();
                        }
                    }
                    if ( robotTimer.seconds() > 0.5 && operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        rightSlide.setPower(-1);
                        leftSlide.setPower(-1);
                    }
                } else {
                    deposit.transitioning();
                    if (operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        rightSlide.setPower(-1);
                        leftSlide.setPower(-1);
                    }
                }
                break;

            case DEPOSIT:
                doClose = false;
                doClamp = false;
                deposit.depositing();
                if(!driver.isDown(GamepadKeys.Button.RIGHT_BUMPER) && !driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                }
                if(operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    rightSlide.setPower(-1);
                    leftSlide.setPower(-1);
                } else if(operator.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    rightSlide.setPower(1);
                    leftSlide.setPower(1);
                } else {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                }
                /*if(robotTimer.milliseconds()>700 && operator!=null) {
                    if(!operator.isDown(GamepadKeys.Button.A)) {
                        state = RETRACT;
                        robotTimer.reset();
                    }
                }*/
                break;

            case RETURNING:
                doClose=false;
                deposit.readyToClamp();
                if (operator.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    claw.looseLeft();
                    claw.looseRight();
                }
                if(leftSlide.getCurrentPosition()>0) {
                    rightSlide.setPower(1);
                    leftSlide.setPower(1);
                } else {
                    rightSlide.setPower(0);
                    leftSlide.setPower(0);
                }
                break;

            case CLIMB:
                rightSlide.setPower(1);
                leftSlide.setPower(1);
                if(robotTimer.seconds()>3) {
                    rightSlide.setPower(0.5);
                    leftSlide.setPower(0.5);
                }
                break;

        }

        telemetry.addData("Left Slide Pos: ", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Pos: ", rightSlide.getCurrentPosition());
        telemetry.addData("Robot Timer: ", robotTimer.seconds());
        telemetry.addData("Current State: ", state);
        telemetry.update();
    }

    public void executeAuto() {
        telemetry.addData("Time: ", robotTimer.seconds());
        if(!(lastState==state)) {
            robotTimer.reset();
            lastState = state;
        }
        switch(state) {
            case START:
                break;
            case REST:
                leftSlide.setPower(0.0);
                rightSlide.setPower(0.0);
                if(!doClamp) {
                    deposit.openLeftClamp();
                    deposit.openRightClamp();
                } else {
                    deposit.closeLeftClamp();
                    deposit.closeRightClamp();
                }
                if(intake.getIntake().getCurrentPosition() < -900) {
                    if (doClose) {
                        claw.closeRight();
                        claw.closeLeft();
                    } else {
                        claw.openRight();
                        claw.openLeft();
                    }
                    if(purpleDeosited) {
                        claw.closeLeft();
                    }
                    if(robotTimer.seconds()>0.5) {
                        intake.intakeManualIn();
                    }
                }
                else { //intake is out
                    arm.transition();
                    wrist.transition();
                    claw.closeRight();
                    claw.closeLeft();
                    if(intake.getIntake().getCurrentPosition() < -40) { //intake is all the way in
                        intake.intakeManualIn();
                    } else {
                        intake.holdIntake();
                    }
                }
                break;

            case PURPLE:
                doClamp = false;
                if(intake.getIntake().getCurrentPosition() > -900) { //intake is out
                    intake.intakeManualOut();
                } else {
                    intake.holdIntake();
                    arm.intake();
                    wrist.intake();
                    claw.closeRight();
                    claw.closeLeft();
                    doClose = false;
                }
                purpleDeosited = true;
                break;

            case SHORT_DEPOSIT:
                doClamp = false;
                if(intake.getIntake().getCurrentPosition() > -900) { //intake is out
                    intake.intakeManualOut();
                } else {
                    intake.holdIntake();
                    arm.shortDeposit();
                    wrist.shortDeposit();
                    doClose = false;
                }
                break;
        }

        telemetry.addData("Left Slide Pos: ", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Pos: ", rightSlide.getCurrentPosition());
        telemetry.addData("Robot Timer: ", robotTimer.seconds());
        telemetry.addData("Current State: ", state);
        telemetry.update();
    }

    public void setPosition(State state){
        //actuator.setPosition(state);
        //slide.setPosition(state);
        //claw.setPosition(state);
        this.state = state;
    }

    public void setState(State state){this.state = state;}
    public State getState(){
        return state;
    }
}
