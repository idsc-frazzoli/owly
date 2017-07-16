// JAVA REFERENCE IMPLEMENTATION OF IMPROVED NOISE - COPYRIGHT 2002 KEN PERLIN.
package ch.ethz.idsc.owly.math.noise;

public enum PerlinNoise implements ContinuousNoise {
  ;
  public static double at(double x, double y, double z) {
    int X = Floor.of(x) & 255; // FIND UNIT CUBE THAT
    int Y = Floor.of(y) & 255; // CONTAINS POINT.
    int Z = Floor.of(z) & 255;
    x -= Math.floor(x); // FIND RELATIVE X,Y,Z
    y -= Math.floor(y); // OF POINT IN CUBE.
    z -= Math.floor(z);
    double u = fade(x); // COMPUTE FADE CURVES
    double v = fade(y); // FOR EACH OF X,Y,Z.
    double w = fade(z);
    int A = PERM[X] + Y; // HASH COORDINATES OF
    int AA = PERM[A] + Z; // THE 8 CUBE CORNERS,
    int AB = PERM[A + 1] + Z;
    int B = PERM[X + 1] + Y;
    int BA = PERM[B] + Z;
    int BB = PERM[B + 1] + Z;
    return lerp(w, lerp(v, lerp(u, grad(PERM[AA], x, y, z), // AND ADD
        grad(PERM[BA], x - 1, y, z)), // BLENDED
        lerp(u, grad(PERM[AB], x, y - 1, z), // RESULTS
            grad(PERM[BB], x - 1, y - 1, z))), // FROM 8
        lerp(v, lerp(u, grad(PERM[AA + 1], x, y, z - 1), // CORNERS
            grad(PERM[BA + 1], x - 1, y, z - 1)), // OF CUBE
            lerp(u, grad(PERM[AB + 1], x, y - 1, z - 1), grad(PERM[BB + 1], x - 1, y - 1, z - 1))));
  }

  private static double fade(double t) {
    return t * t * t * (t * (t * 6 - 15) + 10);
  }

  private static double lerp(double t, double a, double b) {
    return a + t * (b - a);
  }

  private static double grad(int hash, double x, double y, double z) {
    int h = hash & 15; // CONVERT LO 4 BITS OF HASH CODE
    double u = h < 8 ? x : y, // INTO 12 GRADIENT DIRECTIONS.
        v = h < 4 ? y : h == 12 || h == 14 ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
  }
}