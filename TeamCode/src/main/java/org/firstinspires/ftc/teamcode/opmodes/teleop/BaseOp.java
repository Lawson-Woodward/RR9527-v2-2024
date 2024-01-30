package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.State;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import org.firstinspires.ftc.teamcode.commands.VoltageReader;

import java.util.*;

@Config
public abstract class BaseOp extends OpMode {



    public static double armTransition = 0.52, armShortDeposit = 0.68, armIntaking = 0.99;
    public static double wristShortDeposit = 0.53, wristIntaking = 0.64, wristTransition = 0.73;
    public static double rightClawClose = 0.51, rightClawOpen = 0.58;
    public static double leftClawClose = 0.62, leftClawOpen = 0.54;
    public static double planeHoldPos = 0.0, planeReleasePos = 1.0;
    public static double leftClampOpen = 0.5, leftClampClosed = 0.4;
    public static double rightClampOpen = 0.5, rightClampClosed = 0.58;
    public static double depositResting = 0.55, transition = 0.6, extake = 0.28;

    public static boolean doClose = false;

    private ElapsedTime teleTimer = new ElapsedTime();



    private Robot bot;

    private ElapsedTime runtime;
    private GamepadEx driver, operator;
    private VoltageReader voltageReader;
    int alliance;
    boolean canCheckI2C;
    double slideOverride;
    int loop;
    double multiplier, loopTime;
    private boolean curRT, oldRT, curLT, oldLT, tilt, recess, locked;
    int section = 0;
    List<LynxModule> allHubs;

    public abstract void setAlliance(); //-1 BLUE, 0 NEITHER, 1 RED

    public String allianceToString(){
        return alliance == -1 ? "BLUE" : (alliance == 0 ? "NEITHER" : "RED");
    }

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Status: Initializing");
        telemetry.update();

        setAlliance();

        driver = new GamepadEx(gamepad1);   // drives the drivetrain
        operator = new GamepadEx(gamepad2); // controls the scoring systems
        bot = new Robot(hardwareMap, telemetry, driver, operator);
        loop = 0;

        voltageReader = new VoltageReader(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        tilt = true;
        recess = true;

        runtime = new ElapsedTime();

        telemetry.addLine("Status: Initialized");
        telemetry.addLine("Alliance: " + allianceToString());
        telemetry.update();
        bot.setState(State.START);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        multiplier = 1;
        if (loop++ == 10000) loop = 0;

        double startTime = System.currentTimeMillis();

        // ---------------------------- LOOPING ---------------------------- //
        driver.readButtons();
        operator.readButtons();

        // ---------------------------- DRIVER CODE ---------------------------- //
        double desiredSpeed = 0;

        bot.drivetrain.drive(driver);

        if (driver.wasJustPressed(Button.Y)) {                                        // DPad Up = Reset Gyro
            bot.drivetrain.recenter();
        }

        if (driver.getTrigger(Trigger.LEFT_TRIGGER) > 0.1) {                          // Relative speed by dead zone
            desiredSpeed = (0.7 - driver.getTrigger(Trigger.LEFT_TRIGGER) * 0.4) * multiplier;
        } else if (driver.getTrigger(Trigger.RIGHT_TRIGGER) > 0.1) {
            desiredSpeed = 1 * multiplier;
        } else {
            desiredSpeed = 0.7 * multiplier;
        }

        if (driver.isDown(Button.RIGHT_BUMPER)) {
            desiredSpeed *= 0.4;
        }

        if (driver.isDown(Button.LEFT_BUMPER)) {
            bot.drivetrain.switchModes();
        }

        bot.drivetrain.setSpeed(desiredSpeed);


        if (driver.wasJustPressed(Button.DPAD_RIGHT)) {   //RESET BOT
            bot = new Robot(hardwareMap, telemetry, driver, operator);
        }



        if(driver.wasJustPressed(Button.Y)) {
            bot.setState(State.CLIMB);
        }

        /*if (driver.wasJustPressed(Button.A)) {            //uhhh no idea what this does
            tilt = !tilt;
            recess = !recess;
        }*/

        //if right bumper, put the arm thingy down and then the wrist up, and open the claws

        // ---------------------------- OPERATOR CODE ---------------------------- //


        if (driver.wasJustPressed(Button.A)) {
            bot.plane.getPlane().setPosition(0);
        } else if (driver.wasJustPressed(Button.B)) {
            bot.plane.getPlane().setPosition(1);
        }
        bot.executeTele();
        if(operator.isDown(Button.B)) {
            teleTimer.reset();
            bot.setState(State.INTAKE);
        }
        if(operator.isDown(Button.X)) {
            teleTimer.reset();
            bot.setState(State.SHORT_DEPOSIT);
        }
        if(operator.isDown(Button.A)) {
            teleTimer.reset();
            bot.setState(State.DEPOSIT);
        }


        if(operator.isDown(Button.RIGHT_BUMPER)) {                          //SLIDES UP
            bot.setState(State.EXTEND);
        }
        if(operator.isDown(Button.LEFT_BUMPER)) {                    //SLIDES DOWN
            bot.setState(State.RETURNING);
        } else {                                                                    //STOP SLIDES
            //bot.slides.holdSlides();
        }





        /*if(operator.wasJustPressed(Button.B)) {                          //INTAKING
            bot.intake.intakeManualOut();
            timer.reset();
            while (timer.milliseconds()<1000) {}
            bot.intake.holdIntake();
            bot.arm.getArm().setPosition(armIntaking);
            bot.wrist.getWrist().setPosition(wristIntaking);
            timer.reset();
            while(timer.milliseconds()<1000) {}
            bot.claw.getLeftClaw().setPosition(leftClawOpen);
            bot.claw.getRightClaw().setPosition(rightClawOpen);
            doClose = true;
        } else if(operator.wasJustPressed(Button.X)) {
            //SHORT DEPOSIT
            bot.intake.intakeManualOut();
            timer.reset();
            while (timer.milliseconds()<1000) {}
            bot.intake.holdIntake();
            bot.arm.getArm().setPosition(armShortDeposit);
            bot.wrist.getWrist().setPosition(wristShortDeposit);
            doClose = false;
        } else if(operator.wasJustReleased(Button.B) || operator.wasJustReleased(Button.X)) {                                                //TRANSITION
            if(doClose) {
                bot.claw.getRightClaw().setPosition(rightClawClose);
                bot.claw.getLeftClaw().setPosition(leftClawClose);
            } else {
                bot.claw.getRightClaw().setPosition(rightClawOpen);
                bot.claw.getLeftClaw().setPosition(leftClawOpen);
            }
            timer.reset();
            while(timer.milliseconds()<500) {}
            bot.arm.getArm().setPosition(armTransition);
            bot.wrist.getWrist().setPosition(wristTransition);
            bot.claw.getRightClaw().setPosition(rightClawClose);
            bot.claw.getLeftClaw().setPosition(leftClawClose);
            timer.reset();
            while(timer.milliseconds()<500) {}
            bot.intake.intakeManualIn();
            timer.reset();
            while(timer.milliseconds()<1000) {}
            bot.intake.holdIntake();
        }

        if(operator.isDown(Button.RIGHT_BUMPER)) {                          //SLIDES UP
            bot.slides.slidesManualUp();
        } else if(operator.isDown(Button.LEFT_BUMPER)) {                    //SLIDES DOWN
            //bot.slides.getRightSlide().setPower(-1);
            //bot.slides.getLeftSlide().setPower(-1);
            bot.slides.slidesManualDown();
        } else {                                                                    //STOP SLIDES
            bot.slides.holdSlides();
        }

        if(operator.isDown(Button.DPAD_LEFT)) {
            bot.intake.intakeManualIn();
        } else if(operator.isDown(Button.DPAD_RIGHT)) {
            bot.intake.intakeManualOut();
        } else {
            bot.intake.holdIntake();
        }*/







        /*if (operator.wasJustPressed(Button.DPAD_UP)) {
            bot.plane.getPlane().setPosition(planeHoldPos);
        } else if (operator.wasJustPressed(Button.DPAD_DOWN)) {
            bot.plane.getPlane().setPosition(planeReleasePos);
        }*/

        telemetry.addLine("Left Slide Position: " + bot.slides.getLeftPosition());
        telemetry.addLine("Right Slide Position: " + bot.slides.getRightPosition());
        telemetry.addLine("STATE: " + bot.getState());
        telemetry.addLine("is B down: " + operator.isDown(Button.B));
        telemetry.update();

    }
    @Override
    public void stop() {
        super.stop();
    }

    public void teleTelemetry(){
        telemetry.addLine("Runtime: " + runtime.toString());
        telemetry.addLine("Looptime: " + loopTime);
        telemetry.addLine("Multiplier: " + multiplier);
        telemetry.addData("Mode: ", bot.drivetrain.getMode());
    }

    public void hubPowerTelemetry(){
        telemetry.addData("Voltage", voltageReader.getVoltage());
        for(int i = 0; i < allHubs.size(); i++){
            telemetry.addData("Current - Hub" + i, allHubs.get(i).getCurrent(CurrentUnit.AMPS));
        }
    }
}
