// code by jph
package ch.ethz.idsc.owly.gui.region;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.gui.AffineTransforms;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.TensorRank;
import ch.ethz.idsc.tensor.io.ImageFormat;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;

public class ImageRegionRender implements RenderInterface {
  private static final Scalar TFF = RealScalar.of(255);
  private static final Scalar OBS = RealScalar.of(RegionRenders.RGB);
  // ---
  private final BufferedImage bufferedImage;
  private final Tensor scale;
  private final Tensor invsc;

  public ImageRegionRender(ImageRegion imageRegion) {
    Tensor image = imageRegion.image();
    GlobalAssert.that(TensorRank.of(image) == 2);
    bufferedImage = ImageFormat.of(image.map(ImageRegionRender::color));
    scale = imageRegion.scale();
    invsc = DiagonalMatrix.of( //
        +1 / scale.Get(0).number().doubleValue(), //
        -1 / scale.Get(1).number().doubleValue(), 1);
  }

  private static Scalar color(Scalar scalar) {
    return Scalars.isZero(scalar) ? TFF : OBS;
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Tensor model2pixel = geometricLayer.getMatrix();
    Tensor translate = IdentityMatrix.of(3);
    translate.set(RealScalar.of(0), 0, 2);
    translate.set(RealScalar.of(-bufferedImage.getHeight()), 1, 2);
    // GraphicsUtil.setQualityHigh(graphics);
    Tensor matrix = model2pixel.dot(invsc).dot(translate);
    graphics.drawImage(bufferedImage, AffineTransforms.toAffineTransform(matrix), null);
    // GraphicsUtil.setQualityDefault(graphics);
  }
}
