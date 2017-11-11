// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Color;
import java.awt.image.BufferedImage;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.demo.rn.RnPointcloudRegion;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.region.EllipsoidRegion;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.PolygonRegion;
import ch.ethz.idsc.owly.math.region.Region;
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

  /** @param region
   * @return new instance of {@link RenderInterface} that visualizes given region,
   * or null if drawing capability is not available for the region */
  public static RenderInterface create(Region<Tensor> region) {
    if (region instanceof ImageRegion)
      return new ImageRegionRender((ImageRegion) region);
    if (region instanceof EllipsoidRegion)
      return new EllipsoidRegionRender((EllipsoidRegion) region);
    if (region instanceof PolygonRegion)
      return new PolygonRegionRender((PolygonRegion) region);
    if (region instanceof RnPointcloudRegion)
      return new RnPointcloudRegionRender((RnPointcloudRegion) region);
    return null;
  }
}
