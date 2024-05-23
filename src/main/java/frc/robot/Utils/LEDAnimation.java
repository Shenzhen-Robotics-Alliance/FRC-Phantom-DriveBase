package frc.robot.Utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Utils.MathUtils.LookUpTable;
import frc.robot.Utils.MathUtils.StatisticsUtils;

public interface LEDAnimation {
    void play(AddressableLEDBuffer buffer, double t);


    final class Breathe implements LEDAnimation {
        private final int colorR, colorG, colorB;
        private final double hz;
        public Breathe(int colorR, int colorG, int colorB, double hz) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t *= hz;
            final double brightness = 0.5 + 0.5 * Math.sin(t * Math.PI);
            for (int i = 0; i < buffer.getLength(); i++)
                buffer.setRGB(i, (int) (colorR * brightness), (int) (colorG * brightness), (int) (colorB * brightness));
        }
    }

    final class ShowColor implements LEDAnimation {
        private final int colorR, colorG, colorB;
        public ShowColor(int colorR, int colorG, int colorB) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            for (int i = 0; i < buffer.getLength(); i++)
                buffer.setRGB(i, colorR, colorG, colorB);
        }
    }

    final class SlideBackAndForth extends Slide {
        private final double hz1;
        public SlideBackAndForth(int colorR, int colorG, int colorB, double hz, double slideLength) {
            super(colorR, colorG, colorB, 1, slideLength);
            this.hz1 = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            super.play(buffer, 0.5 + 0.5 * Math.sin(t * hz1));
        }
    }

    class Slide implements LEDAnimation {
        private final int colorR, colorG, colorB;
        private final double hz, slideLength;
        public Slide(int colorR, int colorG, int colorB, double hz, double slideLength) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
            this.hz = hz;
            this.slideLength = slideLength;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t *= hz;
            t %= 1;
            final double
                    lowerEdge = LookUpTable.linearInterpretation(0, -slideLength, 1, 1, t),
                    upperEdge = lowerEdge + slideLength;
            EasyShuffleBoard.putNumber("led", "lowerEdge", lowerEdge);
            EasyShuffleBoard.putNumber("led", "upperEdge", upperEdge);
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (i > upperEdge * buffer.getLength() || i < lowerEdge * buffer.getLength()) r = g = b = 0;
                buffer.setRGB(buffer.getLength() / 2 + i, r, g, b);
                buffer.setRGB(buffer.getLength() / 2 - i-1, r, g, b);
            }
        }
    }

    final class Charging implements LEDAnimation {
        private final int colorR, colorG, colorB;
        private final double hz;
        public Charging(int colorR, int colorG, int colorB, double hz) {
            this.colorR = colorR;
            this.colorG = colorG;
            this.colorB = colorB;
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t*= hz;
            final int edge = (int) (t * buffer.getLength() / 2);
            final double coolDownTime = 0.3;

            t *= 1+coolDownTime;
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                int r = colorR, g = colorG, b = colorB;
                if (t > 1) {
                    double brightness = (1+coolDownTime - t) / coolDownTime;
                    r = (int) (r*brightness);
                    g = (int) (g*brightness);
                    b = (int) (b*brightness);
                } else if (i > edge) r = g = b = 0;
                buffer.setRGB(i, r, g, b);
                buffer.setRGB(buffer.getLength() - i-1, r, g, b);
            }
        }
    }

    class ChargingDualColor implements LEDAnimation {
        private final int fromColorR, fromColorG, fromColorB, toColorR, toColorG, toColorB;
        private final double duration;

        public ChargingDualColor(int fromColorR, int fromColorG, int fromColorB, int toColorR, int toColorG, int toColorB, double duration) {
            this.fromColorR = fromColorR;
            this.fromColorG = fromColorG;
            this.fromColorB = fromColorB;
            this.toColorR = toColorR;
            this.toColorG = toColorG;
            this.toColorB = toColorB;
            this.duration = duration;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t = Math.min(Math.max(t/duration, 0), 1);

            // TODO charging animation
        }
    }

    final class Rainbow implements LEDAnimation {
        private final double hz;
        public Rainbow(double hz) {
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t*= hz;
            final int firstPixelHue = (int) (t * 180),
                    v = 128;
            for(var i = 0; i < buffer.getLength()/2; i++) {
                final int colorH = (firstPixelHue + (i * 180 / buffer.getLength())) % 180;
                buffer.setHSV(buffer.getLength()/2 + i, colorH, 255, v);
                buffer.setHSV(buffer.getLength()/2 - i, colorH, 255, v);
            }
        }
    }

    final class PoliceLight implements LEDAnimation {
        private final double hz;
        public PoliceLight(double hz) {
            this.hz = hz;
        }

        @Override
        public void play(AddressableLEDBuffer buffer, double t) {
            t*= hz;
            for (int i = 0; i < buffer.getLength() / 2; i++) {
                final int blink = t > 0.5? 255 : 0;
                buffer.setRGB(i, t > 0.5? 255 : 0, 0, 0);
                buffer.setRGB(buffer.getLength() - i-1, 0, 0,  t < 0.5? 255 : 0);
            }
        }
    }

    final class PreparingToShoot extends ChargingDualColor {

        public PreparingToShoot(double ETA) {
            super(0, 0, 0,
                    230, 255, 0,
                    ETA
            );
        }
    }

    LEDAnimation
            /* light-blue */
            disabled = new LEDAnimation.Breathe(0,200, 255, 0.5),
            enabled = new LEDAnimation.SlideBackAndForth(0,200, 255, 1, 0.8),
            /* purple */
            seeingNote = new LEDAnimation.ShowColor(255, 0, 255),
            searchingNote = new LEDAnimation.Breathe(255, 0, 255, 2),
            /* orange */
            proceedingIntake = new LEDAnimation.Charging(255, 140, 50, 2),
            /* orange to white, then rainbow */
            holdingNote = (buffer, t) -> {
                if (t < 0.6)
                    new ChargingDualColor(255, 140, 50, 255, 255, 255, 0.6).play(buffer, t);
                else
                    new Rainbow(0.6).play(buffer, t);
            },
            /* green */
            searchingSpeaker = new Breathe(0, 255, 0, 2),
            seeingSpeaker = new ShowColor(0, 255, 0),
            /* yellow */
            approachingToSpeaker = new LEDAnimation.Charging(230, 255, 0, 5),
            shooterReady = new LEDAnimation.ShowColor(230, 255, 0),
            /* red */
            armEncoderFailure = new Breathe(255, 0, 0, 5),
            intakeSensorFailure = new ShowColor(255, 0, 0);
}
