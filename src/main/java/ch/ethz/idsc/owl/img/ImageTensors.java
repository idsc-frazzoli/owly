package ch.ethz.idsc.owl.img;

import ch.ethz.idsc.tensor.RealScalar;
import ch.ethz.idsc.tensor.Scalars;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.alg.TensorMap;

public class ImageTensors {
  public static Tensor reduce(Tensor image, Tensor rgba) {
    return TensorMap.of(color -> color.equals(rgba) ? RealScalar.of(255) : RealScalar.ZERO, image, 2);
  }

  public static Tensor reduce(Tensor image, int channel) {
    return TensorMap.of(color -> Scalars.nonZero(color.Get(channel)) ? RealScalar.of(255) : RealScalar.ZERO, image, 2);
  }

  public static Tensor reduceInverted(Tensor image, Tensor rgba) {
    return TensorMap.of(color -> color.equals(rgba) ? RealScalar.ZERO : RealScalar.of(255), image, 2);
  }

  public static Tensor reduceInverted(Tensor image, int channel) {
    return TensorMap.of(color -> Scalars.nonZero(color.Get(channel)) ? RealScalar.ZERO : RealScalar.of(255), image, 2);
  }
}
