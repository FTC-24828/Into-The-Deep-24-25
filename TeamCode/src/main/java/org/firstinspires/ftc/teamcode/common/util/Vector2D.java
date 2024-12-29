package org.firstinspires.ftc.teamcode.common.util;

import android.annotation.SuppressLint;

public class Vector2D {
        public double x, y;

        public Vector2D(double x, double y) {
                this.x = x; this.y = y;
        }

        public Vector2D(double x, double y, double z) {
                this.x = x * Math.cos(-z) + y * -Math.sin(-z);
                this.y = x * Math.sin(-z) + y * Math.cos(-z);
        }

        /**rotates a 2D vector with coordinate (x, y) by z radians*/
        public Vector2D rotate(double z) {
                this.x = x * Math.cos(z) + y * -Math.sin(z);
                this.y = x * Math.sin(z) + y * Math.cos(z);
                return this;
        }

        public Vector2D add(Vector2D v) {
                return new Vector2D(x + v.x, y + v.y);
        }

        public double magnitude() {
                return Math.hypot(x, y);
        }

        public double direction() {
                return Math.atan2(y, x);
        }

        public Vector2D scale(double scalar) {
                return new Vector2D(x * scalar, y * scalar);
        }

        public Vector2D normalize() {
                double h = this.magnitude();
                this.x /= h;
                this.y /= h;
                return this;
        }

        public Vector2D clamp(double min, double max) {
                this.x = WMath.clamp(x, min, max);
                this.y = WMath.clamp(y, min, max);
                return this;
        }

        @SuppressLint("DefaultLocale")
        public String toString() {
                return String.format("x: %.2f, y: %.2f", x, y);
        }
}
