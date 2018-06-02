// code by ynager
package ch.ethz.idsc.owl.math;

import java.io.Serializable;
import java.math.MathContext;

import ch.ethz.idsc.owly.demo.util.Lexicographic;
import ch.ethz.idsc.tensor.AbstractScalar;
import ch.ethz.idsc.tensor.DoubleScalar;
import ch.ethz.idsc.tensor.ExactScalarQ;
import ch.ethz.idsc.tensor.MachineNumberQ;
import ch.ethz.idsc.tensor.Scalar;
import ch.ethz.idsc.tensor.Tensor;
import ch.ethz.idsc.tensor.TensorRuntimeException;
import ch.ethz.idsc.tensor.sca.Chop;
import ch.ethz.idsc.tensor.sca.ChopInterface;
import ch.ethz.idsc.tensor.sca.ExactScalarQInterface;
import ch.ethz.idsc.tensor.sca.MachineNumberQInterface;
import ch.ethz.idsc.tensor.sca.N;
import ch.ethz.idsc.tensor.sca.NInterface;

public class VectorScalar extends AbstractScalar implements //
    ChopInterface, MachineNumberQInterface, NInterface, ExactScalarQInterface, Serializable, Comparable<Scalar> {
  public static Scalar of(Tensor vector) {
    return new VectorScalar(vector.copy());
  }

  private final Tensor vector;

  private VectorScalar(Tensor vector) {
    this.vector = vector;
  }

  public Tensor vector() {
    return vector.copy();
  }

  @Override // from Scalar
  public Scalar multiply(Scalar scalar) {
    return of(vector.multiply(scalar));
  }

  @Override // from Scalar
  public Scalar negate() {
    return of(vector.negate());
  }

  @Override // from Scalar
  public Scalar reciprocal() {
    throw TensorRuntimeException.of(this);
  }

  @Override // from Scalar
  public Scalar abs() {
    return of(vector.map(Scalar::abs));
  }

  @Override // from Scalar
  public Number number() {
    throw TensorRuntimeException.of(this);
  }

  @Override // from Scalar
  public Scalar zero() {
    return of(vector.map(Scalar::zero));
  }

  @Override // from Scalar
  protected Scalar plus(Scalar scalar) {
    if (scalar instanceof VectorScalar) {
      return of(((VectorScalar) scalar).vector.add(vector));
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

  @Override // from Scalar
  public String toString() {
    return vector.toString();
  }

  @Override // from ChopInterface
  public Scalar chop(Chop chop) {
    return of(vector.map(v -> chop.apply(v)));
  }

  // FIXME is this correct?
  @Override // from MachineNumberQInterface
  public boolean isMachineNumber() {
    return !vector.flatten(-1).anyMatch(v -> !MachineNumberQ.of(v));
  }

  @Override // from NInterface
  public Scalar n() {
    return of(vector.map(v -> DoubleScalar.of(v.number().doubleValue())));
  }

  @Override // from NInterface
  public Scalar n(MathContext mathContext) {
    N n = N.in(mathContext.getPrecision());
    return of(vector.map(v -> n.apply(v)));
  }

  @Override // from ExactScalarQInterface
  public boolean isExactScalar() {
    return ExactScalarQ.all(vector);
  }

  @Override // from Comparable
  public int compareTo(Scalar o) {
    if (o instanceof VectorScalar) {
      return Lexicographic.COMPARATOR.compare(vector, ((VectorScalar) o).vector());
    }
    throw TensorRuntimeException.of(this, o);
  }
}
