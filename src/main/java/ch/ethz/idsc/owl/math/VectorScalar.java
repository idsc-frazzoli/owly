package ch.ethz.idsc.owl.math;

import ch.ethz.idsc.tensor.AbstractScalar;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;

public class VectorScalar extends AbstractScalar {
  public static Scalar of(Tensor vector) {
    return new VectorScalar(vector.copy());
  }

  private final Tensor vector;

  private VectorScalar(Tensor vector) {
    this.vector = vector;
  }

  @Override
  public Scalar multiply(Scalar scalar) {
    return new VectorScalar(vector.multiply(scalar));
  }

  @Override
  public Scalar negate() {
    return new VectorScalar(vector.negate());
  }

  @Override
  public Scalar reciprocal() {
    return new VectorScalar(vector.map(Scalar::reciprocal));
  }

  @Override
  public Scalar abs() {
    return new VectorScalar(vector.map(Scalar::abs));
  }

  @Override
  public Number number() {
    throw TensorRuntimeException.of(this);
  }

  @Override
  public Scalar zero() {
    return new VectorScalar(vector.map(Scalar::zero));
  }

  @Override
  protected Scalar plus(Scalar scalar) {
    if (scalar instanceof VectorScalar) {
      return new VectorScalar(((VectorScalar) scalar).vector.add(vector));
    }
    throw TensorRuntimeException.of(this, scalar);
  }

  @Override
  public int hashCode() {
    return vector.hashCode();
  }

  @Override
  public boolean equals(Object object) {
    if (object instanceof VectorScalar) {
      return ((VectorScalar) object).vector.equals(vector);
    }
    return false;
  }

  @Override
  public String toString() {
    return vector.toString();
  }

  public Tensor vector() {
    return vector.copy();
  }
}
