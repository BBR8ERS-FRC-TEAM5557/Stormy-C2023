package org.library.team2910.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
