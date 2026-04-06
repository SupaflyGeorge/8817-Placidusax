package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

/**
 * Utility to batch-refresh all tracked CTRE status signals in one CAN call.
 * Instead of each subsystem calling refreshAll() separately, you register
 * signals here and call StatusSignals.refreshAll() once per loop.
 * (Currently not used in this project but available if needed.)
 */
public class StatusSignals {
    private static List<StatusSignal<?>> statusSignals = new ArrayList<>();
    private static StatusSignal<?>[] statusSignalsArray = null;

    public static <T> StatusSignal<T> trackSignal(StatusSignal<T> statusSignal) {
        statusSignals.add(statusSignal);
        return statusSignal;
    }

    public static void trackSignals(StatusSignal<?>... statusSignals) {
        for (StatusSignal<?> statusSignal : statusSignals) {
            trackSignal(statusSignal);
        }
    }

    public static StatusCode refreshAll() {
        return BaseStatusSignal.refreshAll(getArray());
    }

    private static StatusSignal<?>[] getArray() {
        if (statusSignalsArray == null || statusSignalsArray.length != statusSignals.size()) {
            statusSignalsArray = statusSignals.toArray(new StatusSignal<?>[0]);
        }
        return statusSignalsArray;
    }
}
