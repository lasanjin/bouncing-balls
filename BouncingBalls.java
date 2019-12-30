import javax.swing.*;
import java.awt.*;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.Random;
import java.util.ArrayList;
import java.util.List;

public class BouncingBalls extends JPanel {

    private static final long serialVersionUID = 1L;

    Random rand = new Random();

    private final int frameW = 600;
    private final int frameH = 800;
    private final double g = 0;// 9.82;
    private final double dt = 0.0167; // ~60 fps
    private final double elasticity = 0.8;
    private final int scale = 100; // pixels/meter

    // transformation between physical and graphical screen coordinates
    private final int height = frameW / scale;
    private final int width = frameH / scale;

    List<Ball> balls = new ArrayList<>();

    private void updateBalls() {
        for (Ball b : balls) {
            b.acc();
            b.move();
            // top
            if (b.y - b.radius <= 0) {
                b.y = b.radius;
                b.dy = -b.dy * elasticity;
            }
            // bottom
            if (b.y + b.radius >= height) {
                b.y = height - b.radius;
                b.dy = -b.dy * elasticity;
            }
            // left
            if (b.x - b.radius <= 0) {
                b.x = b.radius;
                b.dx = -b.dx * elasticity;
            }
            // right
            if (b.x + b.radius >= width) {
                b.x = width - b.radius;
                b.dx = -b.dx * elasticity;
            }
        }
    }

    private void updateCollision() {
        int length = balls.size();
        Ball A, B;

        for (int i = 0; i < length; i++) {
            A = balls.get(i);

            for (int j = i + 1; j < length; j++) {
                B = balls.get(j);

                if (distance(A, B) <= A.radius + B.radius) {
                    // collision probably happened dt before distance(A, B) == A.radius + B.radius
                    // so we adjust the distance between center of the two balls -dt time
                    adjustPosition(A, B, -1);
                    // calc new velocities
                    calcCollision(A, B);
                    // because we adjusted time backward dt we need to move time forward dt
                    adjustPosition(A, B, 1);
                }
            }
        }
    }

    private void adjustPosition(Ball A, Ball B, int dir) {
        double disX = (B.x - A.x);
        double disY = (B.y - A.y);
        double dis = distance(A, B);

        // the component of velocity in the direction of (dx,dy)
        double vA = ((A.dx * disX) / dis) + ((A.dy * disY) / dis);
        double vB = ((B.dx * disX) / dis) + ((B.dy * disY) / dis);

        // the time when the two ball really collide with each other.
        double dt = (A.radius + B.radius - dis) / (vA - vB);

        // move balls dt time
        updatePosition(A, B, dt, dir);
    }

    private void updatePosition(Ball A, Ball B, double dt, int dir) {
        if (dt >= 0) {
            A.x += (A.dx * dt) * dir;
            B.x += (B.dx * dt) * dir;
            A.y += (A.dy * dt) * dir;
            B.y += (B.dy * dt) * dir;
        }
    }

    private void calcCollision(Ball A, Ball B) {
        double dis = distance(A, B);
        // unit vectors in direction of collision
        double uvX = (B.x - A.x) / dis;
        double uvY = (B.y - A.y) / dis;

        // projection of the velocities in these axes
        double vAx = (A.dx * uvX) + (A.dy * uvY);
        double vAy = (-A.dx * uvY) + (A.dy * uvX);
        double vBx = (B.dx * uvX) + (B.dy * uvY);
        double vBy = (-B.dx * uvY) + (B.dy * uvX);

        // new velocities in these axes after collision (for elastic collision)
        double vA = vAx + (((1.0 + elasticity) * (vBx - vAx)) / (1.0 + A.m / B.m));
        double vB = vBx + (((1.0 + elasticity) * (vAx - vBx)) / (1.0 + B.m / A.m));

        // update velocity
        A.dx = (vA * uvX) - (vAy * uvY);
        A.dy = (vA * uvY) + (vAy * uvX);
        B.dx = (vB * uvX) - (vBy * uvY);
        B.dy = (vB * uvY) + (vBy * uvX);
    }

    private void createBall(int x, int y) {
        // (x, y, vx, vy, radius, density, color)
        balls.add(new Ball(x / scale, y / scale, randV(), randV(), randR(), 1, randC()));
    }

    private int randV() {
        return (rand.nextInt(20) - 10);
    }

    private double randR() {
        // min + (max - min) * rand.nextDouble()
        return 0.25 + (0.5 - 0.25) * rand.nextDouble();
    }

    private Color randC() {
        return new Color(rand.nextInt(255), rand.nextInt(255), rand.nextInt(255));
    }

    private double distance(Ball A, Ball B) {
        return Math.sqrt(Math.pow((B.x - A.x), 2) + Math.pow((B.y - A.y), 2));
    }

    private double velocity(Ball b) {
        return Math.sqrt(b.dx * b.dx + b.dy * b.dy);
    }

    private double force(Ball b) {
        return Math.sqrt(Math.pow((b.dx / dt), 2) + Math.pow((((b.dy * b.m) / dt) - ((g * b.m))), 2));
    }

    private double sumR() {
        double sum = 0;
        for (Ball b : balls) {
            sum += 2 * b.radius;
        }
        return sum;
    }

    private class Ball {

        private double x; // position
        private double y;
        private double dx; // acc
        private double dy;
        private double Fx; // force
        private double Fy;
        private final double m; // mass
        private final double radius;
        private Color color;

        public Ball(double x, double y, double vx, double vy, double r, double d, Color c) {
            this.x = x;
            this.y = y;
            this.dx = vx;
            this.dy = vy;
            this.radius = r;
            this.m = Math.PI * Math.pow(this.radius, 2) * d;
            this.color = c;
            this.Fx = 0;
            this.Fy = 0;
        }

        public void move() {
            y += dy * dt;
            // x += dx * dt;
            double tmp = dx + ((Fx / m) * dt);
            x += (0.5 * dx + (0.5 * tmp)) * dt; // Runge Kutta
        }

        public void acc() {
            dy += (g + (Fy / m)) * dt;
            // dx += (Fx / m) * dt;
            dx += ((Fx / m) * dt) + (0.5 * (Fx / m) * Math.pow(dt, 2)); // Runge Kutta
        }
    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.WHITE);
        g2d.fillRect(0, 0, frameH, frameW);

        for (Ball ball : balls) {
            drawBall(g2d, ball);
        }
    }

    private void drawBall(Graphics2D g2d, Ball b) {
        //@formatter:off
        Ellipse2D circle = new Ellipse2D.Double(
            (b.x - b.radius) * scale, 
            (b.y - b.radius) * scale,
            2 * b.radius * scale, 
            2 * b.radius * scale);
        // g2d.setColor(b.color);
        double r = b.radius;
        GradientPaint gc = new GradientPaint(
            (float) (b.x - r) * scale, 
            (float) (b.y - r) * scale, 
            b.color, 
            (float) (b.x + r) * scale, 
            (float) (b.y + r) * scale, 
            Color.WHITE,
            true);
        g2d.setPaint(gc);
        g2d.fill(circle);
        //@formatter:on
    }

    private void update() {
        // milliseconds!
        new Timer((int) (dt * 1000), e -> {
            updateBalls();
            updateCollision();
            repaint();
        }).start();
    }

    private void setFrame() {
        setPreferredSize(new Dimension(frameH, frameW));
        JFrame frame = new JFrame();

        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.add(this);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
        frame.addMouseListener(new MouseListener() {
            @Override
            public void mouseClicked(MouseEvent e) {
                // left mouse click adds ball
                if (SwingUtilities.isLeftMouseButton(e)) {
                    // avoid stacking balls on top of each other
                    if (sumR() < width / 2) {
                        createBall(e.getX(), e.getY());
                    }
                }
                // right mouse click removes balls
                else if (SwingUtilities.isRightMouseButton(e)) {
                    balls.clear();
                }
            }

            @Override
            public void mousePressed(MouseEvent e) {
            }

            @Override
            public void mouseReleased(MouseEvent e) {
            }

            @Override
            public void mouseEntered(MouseEvent e) {
            }

            @Override
            public void mouseExited(MouseEvent e) {
            }
        });
    }

    private void run() {
        setFrame();
        update();
    }

    public static void main(String[] args) {
        // enable OpenGL to get accelerated performance in Linux:
        System.setProperty("sun.java2d.opengl", "true");

        new BouncingBalls().run();
    }

}
