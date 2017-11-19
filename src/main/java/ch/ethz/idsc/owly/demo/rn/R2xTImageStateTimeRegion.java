// code by jph
package ch.ethz.idsc.owly.demo.rn;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.function.Supplier;

import ch.ethz.idsc.owly.gui.AffineTransforms;
import ch.ethz.idsc.owly.gui.GeometricLayer;
import ch.ethz.idsc.owly.gui.RenderInterface;
import ch.ethz.idsc.owly.gui.region.RegionRenders;
import ch.ethz.idsc.owly.math.region.ImageRegion;
import ch.ethz.idsc.owly.math.region.Region;
import ch.ethz.idsc.owly.math.se2.RigidFamily;
import ch.ethz.idsc.owly.math.state.StateTime;
import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.mat.DiagonalMatrix;
import ch.ethz.idsc.tensor.mat.IdentityMatrix;
import ch.ethz.idsc.tensor.opt.TensorUnaryOperator;

/** for images only rigid transformations are allowed */
public class R2xTImageStateTimeRegion implements Region<StateTime>, RenderInterface {
  private final ImageRegion imageRegion;
  private final BufferedImage bufferedImage;
  private final RigidFamily rigidFamily;
  private final Supplier<Scalar> supplier;
  private final Tensor invsc;

  /** @param imageRegion
   * @param rigidFamily
   * @param supplier */
  public R2xTImageStateTimeRegion(ImageRegion imageRegion, RigidFamily rigidFamily, Supplier<Scalar> supplier) {
    this.imageRegion = imageRegion;
    bufferedImage = RegionRenders.image(imageRegion.image());
    this.rigidFamily = rigidFamily;
    this.supplier = supplier;
    Tensor scale = imageRegion.scale();
    invsc = DiagonalMatrix.of( //
        scale.Get(0).reciprocal().number().doubleValue(), //
        -scale.Get(1).reciprocal().number().doubleValue(), 1);
  }

  @Override // from Region
  public boolean isMember(StateTime stateTime) {
    Tensor state = stateTime.state().extract(0, 2);
    Scalar time = stateTime.time();
    TensorUnaryOperator rev = rigidFamily.inverse(time);
    return imageRegion.isMember(rev.apply(state));
  }

  @Override // from RenderInterface
  public void render(GeometricLayer geometricLayer, Graphics2D graphics) {
    Scalar time = supplier.get();
    Tensor forward = rigidFamily.forward_se2(time);
    Tensor model2pixel = geometricLayer.getMatrix();
    Tensor translate = IdentityMatrix.of(3);
    translate.set(RealScalar.of(-bufferedImage.getHeight()), 1, 2);
    Tensor matrix = model2pixel.dot(forward).dot(invsc).dot(translate);
    graphics.drawImage(bufferedImage, AffineTransforms.toAffineTransform(matrix), null);
  }
}
