// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.Set;

import ch.ethz.idsc.owly.data.CharImage;
import ch.ethz.idsc.owly.demo.util.FloodFill2D;
import ch.ethz.idsc.owly.demo.util.ImageCostFunction;
import ch.ethz.idsc.owly.glc.core.CostFunction;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.RealScalar;
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

  public static ImageRegion inside_0f5c_2182() {
    CharImage charImage = CharImage.fillWhite(new Dimension(320, 640));
    charImage.setFont(new Font(Font.DIALOG, Font.PLAIN, 600));
    charImage.draw('\u0f5c', new Point(20, 560));
    charImage.setFont(new Font(Font.DIALOG, Font.PLAIN, 270));
    charImage.draw('\u2182', new Point(-5, 230));
    charImage.draw('\u2182', new Point(-5, 420));
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

  public static CharImage inside_roundabout() {
    CharImage charImage = CharImage.fillWhite(new Dimension(236, 180));
    charImage.setFont(new Font(Font.DIALOG, Font.BOLD, 400));
    charImage.draw('a', new Point(-20, 200));
    return charImage;
  }

  private static final Tensor CIRC_RANGE = Tensors.vector(3, 4);

  public static ImageRegion inside_circ() {
    CharImage charImage = inside_roundabout();
    return transpose(charImage.bufferedImage(), CIRC_RANGE, false);
  }

  private static final Tensor GTOB_RANGE = Tensors.vector(12, 12);

  public static ImageRegion inside_gtob() {
    CharImage charImage = inside_gtob_charImage();
    return transpose(charImage.bufferedImage(), GTOB_RANGE, false);
  }

  public static CostFunction imageCost_gtob() throws IOException {
    CharImage charImage = inside_gtob_charImage();
    final Tensor tensor = Transpose.of(ImageFormat.from(charImage.bufferedImage()));
    Set<Tensor> seeds = FloodFill2D.seeds(tensor);
    final int ttl = 15; // magic const
    Tensor cost = FloodFill2D.of(seeds, RealScalar.of(ttl), tensor);
    return new ImageCostFunction(cost.divide(DoubleScalar.of(ttl)), GTOB_RANGE, RealScalar.ZERO);
  }
}
