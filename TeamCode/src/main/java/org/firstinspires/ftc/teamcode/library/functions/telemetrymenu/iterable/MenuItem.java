package org.firstinspires.ftc.teamcode.library.functions.telemetrymenu.iterable;

public abstract class MenuItem<T> {

    abstract String getKey();

    abstract String getDescription();

    public abstract T getValue();

    abstract boolean setValue(T newValue);

    abstract boolean iterateForward();
    abstract boolean iterateBackwards();

    abstract boolean hasNext();
    abstract boolean hasPrevious();


}
