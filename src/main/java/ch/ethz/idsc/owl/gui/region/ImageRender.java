package ch.ethz.idsc.owl.gui.region;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.awt.image.RescaleOp;

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
  public static ImageRender of(BufferedImage image, Tensor range) {
    return new ImageRender(image, range);
  }

  BufferedImage image;
  Tensor invsc;

  public ImageRender(BufferedImage image, Tensor range) {
    this.image = image;
    int dim1 = image.getWidth();
    int dim0 = image.getHeight();
    Tensor scale = Tensors.vector(dim1, dim0).pmul(range.map(Scalar::reciprocal));
    invsc = DiagonalMatrix.of( //
        scale.Get(0).reciprocal().number().doubleValue(), //
        -scale.Get(1).reciprocal().number().doubleValue(), 1);
  }

  public void scaleAlpha(float scale) {
    float[] scales = { 1.0f, 1.0f, 1.0f, scale };
    float[] offsets = { 0.0f, 0.0f, 0.0f, 0.0f };
    RescaleOp op = new RescaleOp(scales, offsets, null);
    image = op.filter(image, null);
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Tensor model2pixel = geometricLayer.getMatrix();
    Tensor translate = IdentityMatrix.of(3);
    translate.set(RealScalar.of(-image.getHeight()), 1, 2);
    Tensor matrix = model2pixel.dot(invsc).dot(translate);
    graphics.drawImage(image, AffineTransforms.toAffineTransform(matrix), null);
  }
}
