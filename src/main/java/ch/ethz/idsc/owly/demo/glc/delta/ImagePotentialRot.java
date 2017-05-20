package ch.ethz.idsc.owly.demo.glc.delta;

import java.io.Serializable;
import java.util.List;

import ch.ethz.idsc.owly.math.SignedCurvature2D;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.opt.Interpolation;
import ch.ethz.idsc.tensor.opt.LinearInterpolation;
import ch.ethz.idsc.tensor.sca.Floor;

/** rotated gradient of potential function */
/* package */ class ImagePotentialRot implements Serializable {
  private final Interpolation interpolation;
  private final List<Integer> dimensions;
  private final Tensor scale;
  private final Scalar amp;

  public ImagePotentialRot(Tensor image, Tensor range, Scalar amp) {
    interpolation = LinearInterpolation.of(image);
    dimensions = Dimensions.of(image);
    scale = Tensors.vector(dimensions).pmul(range.map(Scalar::invert));
    this.amp = amp;
  }

  // TODO check if coordinate rotation !?!?!
  public Tensor at(Tensor tensor) {
    if (tensor.length() != 2)
      tensor = tensor.extract(0, 2);
    Tensor index = Floor.of(tensor.pmul(scale));
    try {
      Scalar f0 = interpolation.Get(index);
      Scalar fx = interpolation.Get(index.add(Tensors.vector(1, 0)));
      Scalar fy = interpolation.Get(index.add(Tensors.vector(0, 1)));
      Scalar dx = fx.subtract(f0);
      Scalar dy = fy.subtract(f0);
      return SignedCurvature2D.cross2d(Tensors.of(dx, dy)).multiply(amp);
    } catch (Exception exception) {
      // ---
    }
    return Tensors.vector(0, 0);
  }
}
