package ch.ethz.idsc.owly.glc.core;

import java.io.Serializable;

import ch.ethz.idsc.tensor.Scalars;

public class CandidatePair implements Comparable<CandidatePair>, Serializable {
  private final GlcNode origin;
  private final GlcNode candidate;

  public CandidatePair(GlcNode origin, GlcNode candidate) {
    this.origin = origin;
    this.candidate = candidate;
  }

//  public int compare(CandidatePair o1, CandidatePair o2) {
//    return Scalars.compare(o1.candidate.merit(), o2.candidate.merit());
//  }

  public GlcNode getOrigin() {
    return origin;
  }

  public GlcNode getCandidate() {
    return candidate;
  }

  @Override
  public int compareTo(CandidatePair o) {
    return Scalars.compare(this.candidate.merit(), o.candidate.merit());
  }
}
