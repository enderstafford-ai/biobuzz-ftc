package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
 * Any FTC-legal controller works without code changes: the SDK sets gamepad1.a for Logitech/Xbox A AND for
 * PlayStation-style ✕ (b = ○, x = □, y = △, bumpers/triggers = L1/R1/L2/R2). Only the names on the Driver Hub
 * screen differ, so screen text writes buttons as {A}, {LT} ... and Pad.text() names them for the controller in use.
 */
final class Pad {
    private static final String[] TOKENS      = {"{A}", "{B}", "{X}", "{Y}", "{LB}", "{RB}", "{LT}", "{RT}"};
    private static final String[] XBOX        = {"A",   "B",   "X",   "Y",   "LB",   "RB",   "LT",   "RT"};
    private static final String[] PLAYSTATION = {"✕",   "○",   "□",   "△",   "L1",   "R1",   "L2",   "R2"};
    private static volatile boolean playStation;

    private Pad() {}

    /** Call every loop: notices which controller is plugged in (it can change mid-match). */
    static void use(Gamepad gamepad) {
        Gamepad.Type t = gamepad.type;
        playStation = t == Gamepad.Type.SONY_PS4 || t == Gamepad.Type.SONY_PS4_SUPPORTED_BY_KERNEL;
    }

    /** Replaces {A}, {LT} ... with this controller's button names. */
    static String text(String s) {
        String[] names = playStation ? PLAYSTATION : XBOX;
        for (int i = 0; i < TOKENS.length; i++) s = s.replace(TOKENS[i], names[i]);
        return s;
    }
}
