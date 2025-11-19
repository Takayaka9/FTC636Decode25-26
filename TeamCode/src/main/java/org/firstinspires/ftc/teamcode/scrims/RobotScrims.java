package org.firstinspires.ftc.teamcode.scrims;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDFControl_ForVelocity;
/*
This file is meant to store all of the information and components on the bot
so we can simply make an instance of this class in other files (such as auto, teleop)
and call each component.
 */
@Configurable
public class RobotScrims {
    public DcMotorEx flyRight, flyLeft, intake; //motor declaration
    public DcMotorEx belt; //idk why this is separate
    public Servo onRamp, offRamp; //servos
    public NormalizedColorSensor colorSensor; //color sensor
    //public DistanceSensor distanceSensor; //distance sensor (same as color)
    Limelight3A limelight3A; //limelight
    public static int onRampPassive = 0; //TODO: test values
    public static int onRampPush = 0;
    public static int offRampPassive = 0;
    public static int offRampPush = 0;
    public static int intakePower = 0;
    public static PIDFControl_ForVelocity shootControl = new PIDFControl_ForVelocity(0.0, 0.0, 0.0, 0.0); //TODO: TUNE PIDF VALUES
    public RobotScrims(HardwareMap hardwareMap){
        flyRight = hardwareMap.get(DcMotorEx.class, "flyRight");
        flyLeft = hardwareMap.get(DcMotorEx.class, "flyLeft");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        belt = hardwareMap.get(DcMotorEx.class, "beltMotor");

        //onRamp = hardwareMap.get(Servo.class, "onRamp");
        //offRamp = hardwareMap.get(Servo.class, "offRamp");

        flyRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flyLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        belt.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        belt.setTargetPosition(belt.getCurrentPosition());
        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

       // colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        //colorSensor.setGain(1);

        //limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

    }

    //initial positions of everything at the start of teleop, as add needed
    public void initialTele(){
        //onRamp.setPosition(onRampPassive);
        //koffRamp.setPosition(offRampPassive);
    }

    public enum DetectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public DetectedColor getDetectedColor(TelemetryManager telemetryManager){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //TODO: calibrate the thresholds and add if statements

        telemetryManager.addData("Red", normRed);
        telemetryManager.addData("Green", normGreen);
        telemetryManager.addData("Blue", normBlue);

        return DetectedColor.UNKNOWN;
    }

    //spins shooter using PIDF control based on the target velocity that is passed through as a parameter
    public void shoot(double velocity){

        double power = shootControl.update(velocity, flyRight.getVelocity());

        flyLeft.setPower(power);
        flyRight.setPower(power);
    }

    //method to push a ball off of the ramp
    /*public void pushOff(){
        onRamp.setPosition(onRampPush);
        onRamp.setPosition(onRampPassive);
    }*/

    //method to push a ball back on the ramp
    /*public void pushOn(){
        offRamp.setPosition(offRampPush);
        offRamp.setPosition(offRampPassive);
    }*/

    //run intake
    public void intakeRun(){
        intake.setPower(intakePower);
    }

    /*checks if a ball is on the distance sensor
    public boolean isBallThere(){
        return distanceSensor.getDistance(DistanceUnit.CM) < 1;
    }
     */

    public void sortMode(String motif){

    }

    //this is Adit's totally not AI code that returns "1" for shoot and "0" for sort based on the three balls that it intakes. dw about it for now.
    /*
    public static String sortToPattern(String s, String preferredPattern) {
        int n = s.length();
        if (n == 0) return ""; // nothing to do

        // Simulate a move sequence; return null if sequence is invalid (shoot when empty)
        java.util.function.Function<String, String[]> simulate = (moves) -> {
            String cur = s;
            StringBuilder out = new StringBuilder();
            for (char m : moves.toCharArray()) {
                if (m == '0') {
                    if (cur.length() > 1) {
                        cur = cur.charAt(cur.length() - 1) + cur.substring(0, cur.length() - 1);
                    } // if cur.length()==1, rotating does nothing (keep as-is)
                } else { // m == '1'
                    if (cur.isEmpty()) return null; // invalid: shooting from empty
                    out.append(cur.charAt(cur.length() - 1));
                    cur = cur.substring(0, cur.length() - 1);
                }
            }
            // valid only if we've shot all original chars (i.e., cur is empty)
            if (!cur.isEmpty()) return null;
            return new String[]{ out.toString(), moves };
        };

        // Score function: number of positions that match preferredPattern (overlapping prefix length)
        java.util.function.BiFunction<String,String,Integer> scoreFn = (out, pref) -> {
            int len = Math.min(out.length(), pref.length());
            int sc = 0;
            for (int i = 0; i < len; i++) if (out.charAt(i) == pref.charAt(i)) sc++;
            return sc;
        };

        String bestMoves = null;
        int bestScore = -1;

        // We must have exactly n '1' operations in the full sequence. Let totalMoves = n + r, where r >= 0
        // We allow some extra rotates (r) up to a small bound. For n<=3, r <= 6 is plenty (keeps search tiny).
        int maxExtraRotates = 6;
        for (int totalMoves = n; totalMoves <= n + maxExtraRotates; totalMoves++) {
            int L = totalMoves;
            int combos = 1 << L; // every move is 0 or 1
            // iterate all bit patterns of length L
            for (int mask = 0; mask < combos; mask++) {
                // quick prune: count ones must equal n
                if (Integer.bitCount(mask) != n) continue;

                // build moves string left-to-right: highest bit is first move or lowest? We'll use bit 0 as first move.
                StringBuilder movesBuilder = new StringBuilder(L);
                for (int i = 0; i < L; i++) {
                    // choose mapping so that '0' preferred lexicographically before '1' naturally
                    boolean bit = ((mask >> i) & 1) == 1;
                    movesBuilder.append(bit ? '1' : '0');
                }
                String moves = movesBuilder.toString();

                // simulate moves
                String[] result = simulate.apply(moves);
                if (result == null) continue;
                String out = result[0];

                int sc = scoreFn.apply(out, preferredPattern);

                // selection: prefer higher score, then fewer moves, then lexicographically smaller (0 before 1)
                if (sc > bestScore
                        || (sc == bestScore && (bestMoves == null || moves.length() < bestMoves.length()))
                        || (sc == bestScore && moves.length() == (bestMoves == null ? Integer.MAX_VALUE : bestMoves.length()) && moves.compareTo(bestMoves) < 0)) {
                    bestScore = sc;
                    bestMoves = moves;
                }
            }
        }

        return bestMoves == null ? "No possible sequence" : bestMoves;
    }*/
}
