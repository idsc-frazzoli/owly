// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.gui.AffineTransforms;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.TensorUnaryOperator;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.se2.RigidFamily;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;

/** check if input tensor is inside a polygon */
public class R2xTImageStateTimeRegion implements Region<StateTime>, RenderInterface {
  private final ImageRegion imageRegion;
  private final RigidFamily rigidFamily;
  private final Supplier<Scalar> supplier;
  private final BufferedImage bufferedImage;
  private final Tensor invsc;

  public R2xTImageStateTimeRegion(ImageRegion imageRegion, RigidFamily bijectionFamily, Supplier<Scalar> supplier) {
    this.imageRegion = imageRegion;
    bufferedImage = RegionRenders.image(imageRegion.image());
    this.rigidFamily = bijectionFamily;
    this.supplier = supplier;
    Tensor scale = imageRegion.scale();
    invsc = DiagonalMatrix.of( //
        scale.Get(0).reciprocal().number().doubleValue(), //
        -scale.Get(1).reciprocal().number().doubleValue(), 1);
  }

  @Override
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, 2);
    Scalar time = stateTime.time();
    TensorUnaryOperator rev = rigidFamily.inverse(time);
    return imageRegion.isMember(rev.apply(state));
  }

  @Override
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    Tensor fwd = rigidFamily.forward_se2(time);
    Tensor model2pixel = geometricLayer.getMatrix();
    Tensor translate = IdentityMatrix.of(3);
    translate.set(RealScalar.of(-bufferedImage.getHeight()), 1, 2);
    Tensor matrix = model2pixel.dot(fwd).dot(invsc).dot(translate);
    graphics.drawImage(bufferedImage, AffineTransforms.toAffineTransform(matrix), null);
  }
}
