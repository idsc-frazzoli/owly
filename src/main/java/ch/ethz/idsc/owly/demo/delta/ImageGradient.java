// code by jph
package ch.ethz.idsc.owly.demo.delta;

import java.io.Serializable;
import java.util.List;

import ch.ethz.idsc.owly.data.GlobalAssert;
import ch.ethz.idsc.owly.math.Cross2D;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.Tensors;
import ch.ethz.idsc.tensor.alg.Array;
import ch.ethz.idsc.tensor.alg.Differences;
import ch.ethz.idsc.tensor.alg.Dimensions;
import ch.ethz.idsc.tensor.alg.MatrixQ;
import ch.ethz.idsc.tensor.alg.Reverse;
import ch.ethz.idsc.tensor.alg.TensorMap;
import ch.ethz.idsc.tensor.alg.Transpose;
import ch.ethz.idsc.tensor.opt.Interpolation;
import ch.ethz.idsc.tensor.opt.LinearInterpolation;
import ch.ethz.idsc.tensor.red.Max;
import ch.ethz.idsc.tensor.red.Norm;
import ch.ethz.idsc.tensor.sca.N;

/** rotated gradient of potential function */
public class ImageGradient implements Serializable {
  private static Tensor _displayOrientation(Tensor tensor) {
    return Transpose.of(Reverse.of(tensor)); // flip y's, then swap x-y
  }

  private static final Tensor ZEROS = N.of(Array.zeros(2));
  // ---
  private final Tensor scale;
  private final Interpolation interpolation;
  private final Scalar maxNorm;

  /** @param image with rank 2. For instance, Dimensions.of(image) == [179, 128]
   * @param range with length() == 2
   * @param amp factor */
  public ImageGradient(Tensor _image, Tensor range, Scalar amp) {
    Tensor image = _displayOrientation(_image);
    GlobalAssert.that(MatrixQ.of(image));
    List<Integer> dims = Dimensions.of(image);
    scale = Tensors.vector(dims).pmul(range.map(Scalar::reciprocal));
    Tensor diffx = Differences.of(image);
    diffx = TensorMap.of(t -> t.extract(0, dims.get(1) - 1), diffx, 1);
    Tensor diffy = Transpose.of(Differences.of(Transpose.of(image)));
    diffy = diffy.extract(0, dims.get(0) - 1);
    Tensor field = Transpose.of(Tensors.of(diffx, diffy), 2, 0, 1);
    field = TensorMap.of(Cross2D::of, field, 2).multiply(amp);
    field = N.of(field);
    interpolation = LinearInterpolation.of(field);
    maxNorm = field.flatten(1).map(Norm._2::ofVector).reduce(Max::of).get();
  }

  public Tensor rotate(Tensor tensor) {
    if (tensor.length() != 2)
      tensor = tensor.extract(0, 2);
    Tensor index = tensor.pmul(scale);
    try {
      return interpolation.get(index);
    } catch (Exception exception) {
      // ---
    }
    return ZEROS.copy();
  }

  /** @return max(||gradient||) */
  public Scalar maxNorm() {
    return maxNorm;
  }
}
