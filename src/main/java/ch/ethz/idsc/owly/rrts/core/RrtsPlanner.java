// code by jph
package ch.ethz.idsc.owly.rrts.core;

public class RrtsPlanner implements ExploreInterface {
  private final Rrts rrts;
  private final TransitionRegionQuery obstacleQuery;
  private final SamplerInterface spaceSampler;
  private final SamplerInterface goalSampler;

  /** @param rrts
   * @param obstacleQuery
   * @param spaceSampler
   * @param goalSampler generates samples in goal region */
  public RrtsPlanner( //
      Rrts rrts, TransitionRegionQuery obstacleQuery, //
      SamplerInterface spaceSampler, SamplerInterface goalSampler) {
    this.rrts = rrts;
    this.obstacleQuery = obstacleQuery;
    this.spaceSampler = spaceSampler;
    this.goalSampler = goalSampler;
  }
}
