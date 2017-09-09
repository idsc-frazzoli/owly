// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.Point;
import java.awt.image.BufferedImage;

import ch.ethz.idsc.owly.data.CharImage;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.io.ImageFormat;

/** collection of ready-to-use image regions */
public enum R2ImageRegions {
  ;
  private static ImageRegion transpose(BufferedImage bufferedImage, Tensor range, boolean strict) {
    return new ImageRegion(Transpose.of(ImageFormat.from(bufferedImage)), range, strict);
  }

  public static ImageRegion outside_0b36() {
    CharImage charImage = CharImage.fillBlack(new Dimension(256, 256));
    charImage.setFont(new Font(Font.DIALOG, Font.PLAIN, 256));
    charImage.draw('\u0b36', new Point(30, 200));
    return transpose(charImage.bufferedImage(), Tensors.vector(7, 7), false);
  }

  public static ImageRegion inside_0b36() {
    CharImage charImage = CharImage.fillWhite(new Dimension(210, 256));
    charImage.draw('\u0b36', new Point(0, 240));
    return transpose(charImage.bufferedImage(), Tensors.vector(6, 7), false);
  }

  public static ImageRegion inside_265b() {
    CharImage charImage = CharImage.fillWhite(new Dimension(320, 320));
    charImage.draw('\u265b', new Point(-20, 300));
    return transpose(charImage.bufferedImage(), Tensors.vector(7, 7), false);
  }

  public static ImageRegion inside_2182() {
    CharImage charImage = CharImage.fillWhite(new Dimension(480, 320));
    charImage.draw('\u2182', new Point(-10, 305));
    return transpose(charImage.bufferedImage(), Tensors.vector(9, 6), false);
  }

  public static ImageRegion inside_0f5c() {
    CharImage charImage = CharImage.fillWhite(new Dimension(320, 640));
    charImage.setFont(new Font(Font.DIALOG, Font.PLAIN, 600));
    charImage.draw('\u0f5c', new Point(20, 560));
    return transpose(charImage.bufferedImage(), Tensors.vector(20, 10), false);
  }

  public static CharImage inside_gtob_charImage() {
    CharImage charImage = CharImage.fillWhite(new Dimension(640, 640));
    charImage.setFont(new Font(Font.DIALOG, Font.BOLD, 400));
    charImage.draw('G', new Point(0, 310));
    charImage.draw('T', new Point(280, 323));
    charImage.draw('I', new Point(480, 323));
    charImage.draw('O', new Point(20, 560));
    charImage.draw('B', new Point(280, 580));
    return charImage;
  }

  public static ImageRegion inside_gtob() {
    CharImage charImage = inside_gtob_charImage();
    return transpose(charImage.bufferedImage(), Tensors.vector(12, 12), false);
  }
}
