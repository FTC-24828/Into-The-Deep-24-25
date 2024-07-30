package org.firstinspires.ftc.teamcode.common.util;

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

        public Vector2D Add(Vector2D v) {
                return new Vector2D(x + v.x, y + v.y);
        }

        public double magnitude() {
                return Math.hypot(x, y);
        }

        public Vector2D scale(double scalar) {
                this.x *= scalar;
                this.y *= scalar;
                return this;
        }

        public Vector2D clamp(double min, double max) {
                this.x = WMath.clamp(x, min, max);
                this.y = WMath.clamp(y, min, max);
                return this;
        }
}
