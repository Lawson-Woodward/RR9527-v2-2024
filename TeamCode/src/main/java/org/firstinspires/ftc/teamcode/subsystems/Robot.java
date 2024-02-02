package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
    public boolean doClose = true, doClamp = false, doClimb = false;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public Deposit deposit;
    public LinearSlides slides;
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
        slides = new LinearSlides(hardwareMap);
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
        slides = new LinearSlides(hardwareMap);
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
                if(robotTimer.seconds()>700) {
                    deposit.transitioning();
                }
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
                claw.looseLeft();
                claw.looseRight();
                if(slides.getLeftSlide().getCurrentPosition()<150) {
                    if (robotTimer.seconds() > 0.2 && robotTimer.seconds() < 0.5) {
                        deposit.readyToClamp();
                    }
                    if(doClamp) {
                        if (robotTimer.seconds() > 0.5 && robotTimer.seconds() < 0.8) {
                            deposit.closeLeftClamp();
                            deposit.closeRightClamp();
                        }
                    }
                    if ( robotTimer.seconds() > 0.8 && operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        slides.slidesManualUp();
                    }
                } else {
                    deposit.transitioning();
                    if (operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                        slides.slidesManualUp();
                    }
                }
                break;

            case DEPOSIT:
                doClamp = false;
                deposit.depositing();
                if(robotTimer.milliseconds()>700 && operator!=null) {
                    if(!operator.isDown(GamepadKeys.Button.A)) {
                        state = RETRACT;
                        robotTimer.reset();
                    }
                }
                break;

            case RETRACT:
                //

            case RETURNING:
                doClose=true;
                if(slides.getLeftSlide().getCurrentPosition()<250) {
                    deposit.transitioning();
                }
                if (operator.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    slides.slidesManualDown();
                }
                break;

            case CLIMB:
                doClimb = true;
                break;
            case LAUNCH_DRONE:
                //
                break;

        }

        if(doClimb) {
            slides.slidesManualDown();
            if(robotTimer.seconds()>3) {
                slides.getRightSlide().setPower(0.6);
                slides.getRightSlide().setPower(0.6);
            }
        }

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
