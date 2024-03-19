package org.firstinspires.ftc.teamcode.wilyworks;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.Iterator;
import java.util.Spliterator;
import java.util.function.Consumer;

public class WilyHardwareMap implements Iterable<HardwareDevice> {
    @NonNull
    @Override
    public Iterator<HardwareDevice> iterator() {
        return null;
    }

    @Override
    public void forEach(@NonNull Consumer<? super HardwareDevice> action) {
        Iterable.super.forEach(action);
    }

    @NonNull
    @Override
    public Spliterator<HardwareDevice> spliterator() {
        return Iterable.super.spliterator();
    }
}
