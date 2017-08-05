// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.Point;

import ch.ethz.idsc.owly.data.CharImage;
import ch.ethz.idsc.owly.demo.util.ImageRegions;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensors;

public enum R2ImageRegions {
  ;
  public static ImageRegion outside_0b36() {
    CharImage charImage = CharImage.fillBlack(new Dimension(256, 256));
    charImage.setFont(new Font(Font.DIALOG, Font.PLAIN, 256));
    charImage.draw('\u0b36', new Point(30, 200));
    return ImageRegions.fromGrayscale(charImage.getBufferedImage(), Tensors.vector(7, 7), false);
  }

  public static ImageRegion inside_0b36() {
    CharImage charImage = CharImage.fillWhite(new Dimension(210, 256));
    charImage.draw('\u0b36', new Point(0, 240));
    return ImageRegions.fromGrayscale(charImage.getBufferedImage(), Tensors.vector(6, 7), false);
  }

  public static ImageRegion inside_265b() {
    CharImage charImage = CharImage.fillWhite(new Dimension(320, 320));
    charImage.draw('\u265b', new Point(-20, 300));
    return ImageRegions.fromGrayscale(charImage.getBufferedImage(), Tensors.vector(7, 7), false);
  }

  public static ImageRegion inside_2182() {
    CharImage charImage = CharImage.fillWhite(new Dimension(480, 320));
    charImage.draw('\u2182', new Point(-10, 305));
    return ImageRegions.fromGrayscale(charImage.getBufferedImage(), Tensors.vector(9, 6), false);
  }
}
