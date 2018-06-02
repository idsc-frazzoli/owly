// code by ynager
package ch.ethz.idsc.owl.gui.region;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;

import ch.ethz.idsc.owl.gui.AffineTransforms;
import ch.ethz.idsc.owl.gui.GeometricLayer;
import ch.ethz.idsc.owl.gui.RenderInterface;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;

public class ImageRender implements RenderInterface {
  private final BufferedImage image;
  private final Tensor matrix;

  public ImageRender(BufferedImage image, Tensor range) {
    this.image = image;
    int dim1 = image.getWidth();
    int dim0 = image.getHeight();
    Tensor scale = Tensors.vector(dim1, dim0).pmul(range.map(Scalar::reciprocal));
    Tensor invsc = DiagonalMatrix.of( //
        scale.Get(0).reciprocal().number().doubleValue(), //
        -scale.Get(1).reciprocal().number().doubleValue(), 1);
    Tensor translate = IdentityMatrix.of(3);
    translate.set(RealScalar.of(-image.getHeight()), 1, 2);
    matrix = invsc.dot(translate);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    graphics.drawImage(image, //
        AffineTransforms.toAffineTransform(geometricLayer.getMatrix().dot(matrix)), null);
  }
}
