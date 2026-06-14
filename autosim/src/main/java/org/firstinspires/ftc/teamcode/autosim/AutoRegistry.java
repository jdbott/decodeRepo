package org.firstinspires.ftc.teamcode.autosim;

import org.firstinspires.ftc.teamcode.autosim.autos.V3AutoSim;
import org.firstinspires.ftc.teamcode.autosim.autos.V3ClosePartnerSim;
import org.firstinspires.ftc.teamcode.autosim.autos.V3FarAutoSim;
import org.firstinspires.ftc.teamcode.autosim.model.SimTrace;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Registry of simulatable autos. Each entry is one auto that has a {@code ...Sim} copy; the
 * generator builds every registered trace into the viewer so the dropdown can switch between
 * them. Adding a new auto to the sim is a single line here.
 */
public final class AutoRegistry {
    private AutoRegistry() {}

    /** Insertion order = dropdown order; the first entry is the default selection. */
    public static final Map<String, Supplier<SimTrace>> AUTOS = new LinkedHashMap<>();
    static {
        AUTOS.put("V3FarAuto", V3FarAutoSim::build);
        AUTOS.put("V3ClosePartner", V3ClosePartnerSim::build);
        AUTOS.put("V3Auto", V3AutoSim::build);
    }
}
