// code by jph
package ch.ethz.idsc.owly.demo.glc.delta;

import java.io.Serializable;
import java.util.List;

import ch.ethz.idsc.owly.math.Cross2D;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.opt.Interpolation;
import ch.ethz.idsc.tensor.opt.LinearInterpolation;

/** rotated gradient of potential function */
/* package */ class ImageGradient implements Serializable {
  private static final Tensor ZEROS = Array.zeros(2);
  private static final Tensor DUX = Tensors.vector(1, 0);
  private static final Tensor DUY = Tensors.vector(0, 1);
  // ---
  private final Interpolation interpolation;
  private final List<Integer> dimensions;
  private final Tensor scale;
  private final Scalar amp;

  /** @param image
   * @param range with length() == 2
   * @param amp */
  public ImageGradient(Tensor image, Tensor range, Scalar amp) {
    interpolation = LinearInterpolation.of(image);
    dimensions = Dimensions.of(image);
    scale = Tensors.vector(dimensions).pmul(range.map(Scalar::invert));
    this.amp = amp;
  }

  public Tensor rotate(Tensor tensor) {
    if (tensor.length() != 2)
      tensor = tensor.extract(0, 2);
    Tensor index = tensor.pmul(scale);
    try {
      Scalar f0 = interpolation.Get(index);
      Scalar fx = interpolation.Get(index.add(DUX));
      Scalar fy = interpolation.Get(index.add(DUY));
      Scalar dx = fx.subtract(f0);
      Scalar dy = fy.subtract(f0);
      return Cross2D.of(Tensors.of(dx, dy)).multiply(amp);
    } catch (Exception exception) {
      // ---
    }
    return ZEROS;
  }

  public Tensor getMax() {
    // TODO return max(||gradient||)
    return null;
  }
}
