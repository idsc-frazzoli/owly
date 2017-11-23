// code by jl
package ch.ethz.idsc.owly.glc.any;

import java.io.Serializable;

import ch.ethz.idsc.owly.glc.core.GlcNode;
import ch.ethz.idsc.tensor.Scalars;

/** CandidatePair is used in HashSet */
/* package */ public class CandidatePair implements Comparable<CandidatePair>, Serializable {
  private final GlcNode origin;
  private final GlcNode candidate;

  public CandidatePair(GlcNode origin, GlcNode candidate) {
    this.origin = origin;
    this.candidate = candidate;
  }

  public GlcNode getOrigin() {
    return origin;
  }

  public GlcNode getCandidate() {
    return candidate;
  }

  @Override
  public int compareTo(CandidatePair candidatePair) {
    return Scalars.compare(candidate.merit(), candidatePair.candidate.merit());
  }
}
