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
    private boolean climb = false;


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
            bot.setState(State.CLIMB);
        }

        if (driver.getTrigger(Trigger.LEFT_TRIGGER) > 0.1) {                          // Relative speed by dead zone
            desiredSpeed = (0.7 - driver.getTrigger(Trigger.LEFT_TRIGGER) * 0.4) * multiplier;
        } else if (driver.getTrigger(Trigger.RIGHT_TRIGGER) > 0.1) {
            desiredSpeed = 1 * multiplier;
        } else {
            desiredSpeed = 0.7 * multiplier;
        }

        if (driver.isDown(Button.B)) {
            desiredSpeed *= 0.5;
        }

        if (driver.wasJustPressed(Button.X)) {
            bot.drivetrain.switchModes();
        }

        bot.drivetrain.setSpeed(desiredSpeed);


        if (driver.wasJustPressed(Button.DPAD_RIGHT)) {   //RESET BOT
            bot = new Robot(hardwareMap, telemetry, driver, operator);
        }


        if (operator.wasJustPressed(Button.X)) {
            bot.plane.getPlane().setPosition(0);
        } else if (operator.wasJustPressed(Button.Y)) {
            bot.plane.getPlane().setPosition(1);
        }

        // ---------------------------- OPERATOR CODE ---------------------------- //

        if(operator.isDown(Button.A)) {
            bot.setState(State.DEPOSIT);
        } else if(driver.isDown(Button.RIGHT_BUMPER)) {
            bot.setState(State.INTAKE);
        } else if(driver.isDown(Button.LEFT_BUMPER)) {
            bot.setState(State.SHORT_DEPOSIT);
        } else if(operator.isDown(Button.RIGHT_BUMPER)){
            bot.setState(State.EXTEND);
        } else if(operator.isDown(Button.LEFT_BUMPER)) {
            bot.setState(State.RETURNING);
        } else {
            if (climb) {
                bot.setState(State.CLIMB);
            } else {
                bot.setState(State.REST);
            }
        }

        if (driver.isDown(Button.Y)) {
            climb = true;
        }

        bot.executeTele();

        //telemetry.addLine("Left Slide Position: " + bot.slides.getLeftPosition());
        //telemetry.addLine("Right Slide Position: " + bot.slides.getRightPosition());
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
