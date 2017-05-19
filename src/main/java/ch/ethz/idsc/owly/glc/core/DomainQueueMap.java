// code by jph
package ch.ethz.idsc.owly.glc.core;

import java.util.HashMap;
import java.util.Map;

import ch.ethz.idsc.tensor.Tensor;

/* package */ class DomainQueueMap {
  final Map<Tensor, DomainQueue> map = new HashMap<>();

  /** add given glcNode to domain queue at the location determined by domain_key.
   * a new domain queue is allocated if no other nodes were inserted there prior.
   * 
   * @param domain_key
   * @param glcNode */
  public void insert(Tensor domain_key, GlcNode glcNode) {
    if (!map.containsKey(domain_key)) // has another candidate has already reached this domain ?
      map.put(domain_key, new DomainQueue()); // if not, create a new empty queue
    map.get(domain_key).add(glcNode); // <- add node to queue (always)
  }
}
