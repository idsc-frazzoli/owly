// code by ynager
package ch.ethz.idsc.owl.math;

import java.io.Serializable;
import java.math.MathContext;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import ch.ethz.idsc.tensor.AbstractScalar;
import ch.ethz.idsc.tensor.ExactScalarQ;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.ChopInterface;
import ch.ethz.idsc.tensor.sca.ExactScalarQInterface;
import ch.ethz.idsc.tensor.sca.N;
import ch.ethz.idsc.tensor.sca.NInterface;

/** immutable
 * 
 * string expression is of the form [1, 2, 3] to prevent confusion with standard vectors */
public class VectorScalar extends AbstractScalar implements //
    ChopInterface, ExactScalarQInterface, NInterface, Comparable<Scalar>, Serializable {
  /** @param vector
   * @return
   * @throws Exception if input is not a vector, or contains entries of type {@link VectorScalar} */
  public static Scalar of(Tensor vector) {
    if (vector.stream().map(Scalar.class::cast).anyMatch(VectorScalar.class::isInstance))
      throw TensorRuntimeException.of(vector);
    return new VectorScalar(vector.copy());
  }

  // ---
  private final Tensor vector;

  private VectorScalar(Tensor vector) {
    this.vector = vector;
  }

  /** @return copy of content vector */
  public Tensor vector() {
    return vector.copy();
  }

  @Override // from Scalar
  public Scalar multiply(Scalar scalar) {
    return new VectorScalar(vector.multiply(scalar));
  }

  @Override // from Scalar
  public Scalar negate() {
    return new VectorScalar(vector.negate());
  }

  @Override // from Scalar
  public Scalar reciprocal() {
    throw TensorRuntimeException.of(this);
  }

  @Override // from Scalar
  public Scalar abs() {
    return new VectorScalar(vector.map(Scalar::abs));
  }

  @Override // from Scalar
  public Number number() {
    throw TensorRuntimeException.of(this);
  }

  @Override // from Scalar
  public Scalar zero() {
    return new VectorScalar(vector.map(Scalar::zero));
  }

  /***************************************************/
  @Override // from AbstractScalar
  protected Scalar plus(Scalar scalar) {
    if (scalar instanceof VectorScalar) {
      VectorScalar vectorScalar = (VectorScalar) scalar;
      return new VectorScalar(vector.add(vectorScalar.vector));
    }
    throw TensorRuntimeException.of(this, scalar);
  }

  /***************************************************/
  @Override // from ChopInterface
  public Scalar chop(Chop chop) {
    return new VectorScalar(chop.of(vector));
  }

  @Override // from NInterface
  public Scalar n() {
    return new VectorScalar(N.DOUBLE.of(vector));
  }

  @Override // from NInterface
  public Scalar n(MathContext mathContext) {
    return new VectorScalar(N.in(mathContext.getPrecision()).of(vector));
  }

  @Override // from ExactScalarQInterface
  public boolean isExactScalar() {
    return ExactScalarQ.all(vector);
  }

  /***************************************************/
  @Override // from Comparable
  public int compareTo(Scalar scalar) {
    if (scalar instanceof VectorScalar) {
      VectorScalar vectorScalar = (VectorScalar) scalar;
      return Lexicographic.COMPARATOR.compare(vector, vectorScalar.vector());
    }
    throw TensorRuntimeException.of(this, scalar);
  }

  @Override // from Scalar
  public int hashCode() {
    return vector.hashCode();
  }

  @Override // from Scalar
  public boolean equals(Object object) {
    if (object instanceof VectorScalar) {
      return ((VectorScalar) object).vector.equals(vector);
    }
    return false;
  }

  private static final Collector<CharSequence, ?, String> EMBRACE = Collectors.joining(", ", "[", "]");

  @Override // from Scalar
  public String toString() {
    return vector.stream().map(Tensor::toString).collect(EMBRACE);
  }
}
