// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Color;
import java.awt.image.BufferedImage;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.TensorRank;
import ch.ethz.idsc.tensor.io.ImageFormat;

public enum RegionRenders {
  ;
  public static final int RGB = 230;
  /** default color for obstacle region */
  public static final Color COLOR = new Color(RGB, RGB, RGB);
  public static final Color BOUNDARY = new Color(192, 192, 192);
  // ---
  private static final Scalar TFF = RealScalar.of(255);
  private static final Scalar OBS = RealScalar.of(RegionRenders.RGB);

  private static Scalar color(Scalar scalar) {
    return Scalars.isZero(scalar) ? TFF : OBS;
  }

  public static BufferedImage image(Tensor image) {
    GlobalAssert.that(TensorRank.of(image) == 2);
    return ImageFormat.of(image.map(RegionRenders::color));
  }
}
