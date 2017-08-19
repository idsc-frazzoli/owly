// code by jph
package ch.ethz.idsc.owly.gui.misc;

import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.awt.image.ImageObserver;

import javax.swing.JLabel;

import ch.ethz.idsc.owly.gui.OwlyLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.io.ImageFormat;

public class ImageRegionRender implements RenderInterface {
  private static final Scalar TFF = RealScalar.of(255);
  private static final Scalar OBS = RealScalar.of(230);
  private static final ImageObserver OBSERVER = new JLabel();
  // ---
  private final BufferedImage bufferedImage;
  private final Tensor range;

  public ImageRegionRender(ImageRegion imageRegion) {
    Tensor image = imageRegion.image();
    bufferedImage = ImageFormat.of(image.map(ImageRegionRender::color));
    this.range = imageRegion.range();
  }

  private static Scalar color(Scalar scalar) {
    if (Scalars.isZero(scalar))
      return TFF;
    return OBS;
  }

  @Override
  public void render(OwlyLayer owlyLayer, Graphics2D graphics) {
    Point2D origin = owlyLayer.toPoint2D(Tensors.of(RealScalar.ZERO, range.Get(1)));
    Point2D uprigh = owlyLayer.toPoint2D(Tensors.of(range.Get(0), RealScalar.ZERO));
    int width = (int) (uprigh.getX() - origin.getX());
    int height = (int) (uprigh.getY() - origin.getY());
    graphics.drawImage(bufferedImage, //
        (int) origin.getX(), //
        (int) origin.getY(), //
        width, height, OBSERVER);
  }
}
