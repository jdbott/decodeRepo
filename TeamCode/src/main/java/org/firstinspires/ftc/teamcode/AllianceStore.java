package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.content.Context;

public class AllianceStore {

    private static final String KEY = "alliance"; // stored value
    private static final String RED = "RED";
    private static final String BLUE = "BLUE";

    // Setters
    public static void setRed(Context context) {
        SharedPreferences prefs =
                PreferenceManager.getDefaultSharedPreferences(context);
        prefs.edit().putString(KEY, RED).apply();
    }

    public static void setBlue(Context context) {
        SharedPreferences prefs =
                PreferenceManager.getDefaultSharedPreferences(context);
        prefs.edit().putString(KEY, BLUE).apply();
    }

    // Getters
    public static boolean isRed(Context context) {
        SharedPreferences prefs =
                PreferenceManager.getDefaultSharedPreferences(context);
        return prefs.getString(KEY, BLUE).equals(RED); // default = BLUE
    }

    public static boolean isBlue(Context context) {
        return !isRed(context);
    }

    public static String get(Context context) {
        SharedPreferences prefs =
                PreferenceManager.getDefaultSharedPreferences(context);
        return prefs.getString(KEY, BLUE);
    }
}