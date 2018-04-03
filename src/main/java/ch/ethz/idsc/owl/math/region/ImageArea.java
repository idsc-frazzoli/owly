// code by ynager, found on github
package ch.ethz.idsc.owl.math.region;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.geom.Area;
import java.awt.image.BufferedImage;

public class ImageArea extends Area {
  public ImageArea(BufferedImage image, Color color, int tolerance) {
    super(getArea(image, color, tolerance));
  }

  public static Area getArea(BufferedImage image, Color color, int tolerance) {
    if (image == null)
      return null;
    Area area = new Area();
    for (int x = 0; x < image.getWidth(); x++) {
      for (int y = 0; y < image.getHeight(); y++) {
        Color pixel = new Color(image.getRGB(x, y));
        if (isIncluded(color, pixel, tolerance)) {
          Rectangle r = new Rectangle(x, y, 1, 1);
          area.add(new Area(r));
        }
      }
    }
    return area;
  }

  private static boolean isIncluded(Color target, Color pixel, int tolerance) {
    int rT = target.getRed();
    int gT = target.getGreen();
    int bT = target.getBlue();
    int rP = pixel.getRed();
    int gP = pixel.getGreen();
    int bP = pixel.getBlue();
    return ((rP - tolerance <= rT) && (rT <= rP + tolerance) && //
        (gP - tolerance <= gT) && (gT <= gP + tolerance) && //
        (bP - tolerance <= bT) && (bT <= bP + tolerance));
  }
}
