package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

public class AutoStartStore {
    private static final String PREFS_NAME = "RobotPrefs";
    private static final String KEY_AUTO_START = "auto_start_mode";

    public static final String CLOSE = "CLOSE";
    public static final String FAR = "FAR";

    public static void setClose(Context context) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .putString(KEY_AUTO_START, CLOSE)
                .apply();
    }

    public static void setFar(Context context) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .edit()
                .putString(KEY_AUTO_START, FAR)
                .apply();
    }

    public static String get(Context context) {
        return context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
                .getString(KEY_AUTO_START, CLOSE);
    }

    public static boolean isClose(Context context) {
        return CLOSE.equalsIgnoreCase(get(context));
    }

    public static boolean isFar(Context context) {
        return FAR.equalsIgnoreCase(get(context));
    }
}