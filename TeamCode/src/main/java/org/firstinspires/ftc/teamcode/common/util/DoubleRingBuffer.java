package org.firstinspires.ftc.teamcode.common.util;

import java.lang.reflect.Array;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;

public class DoubleRingBuffer {
    private final ArrayList<double[]> ring;
    public int capacity;
    private int index = 0;

    public DoubleRingBuffer(int capacity) {
        this.capacity = capacity;
        this.ring = new ArrayList<>(capacity);
    }

    public boolean isFull() {
        return ring.size() == capacity;
    }

    public boolean isEmpty() {
        return ring.isEmpty();
    }

    public void add(double[] v) {
        index++;
        index = index % capacity;
        ring.set(index, v);
    }

    public double[] get(int i) {
        i = (index - i) % capacity;
        return ring.get(i);
    }

    @Override
    public String toString() {
        return "RingBufferWithArrayDeque{" +
                "ring=" + ring +
                ", capacity=" + capacity +
                '}';
    }
}
