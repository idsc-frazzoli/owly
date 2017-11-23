// code by jl
package ch.ethz.idsc.owly.glc.any;

import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.tensor.Tensor;

/* package */ class CandidatePairQueueMap implements Serializable {
  final Map<Tensor, CandidatePairQueue> map = new HashMap<>();

  /** add given candidate to CandidatePair queue at the location determined by domain_key.
   * a new domain queue is allocated if no other nodes were inserted there prior.
   * 
   * @param domain_key
   * @param candidatePair */
  public void insert(Tensor domain_key, CandidatePair candidatePair) {
    if (!map.containsKey(domain_key)) // has another candidate has already reached this domain ?
      map.put(domain_key, new CandidatePairQueue()); // if not, create a new empty queue
    map.get(domain_key).add(candidatePair); // <- add node to queue (always)
  }
}
