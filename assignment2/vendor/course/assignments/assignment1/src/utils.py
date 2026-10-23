import numpy as np

from data_types.nominal_types import vct3


def sample_normal(mean, cov, n=1):
    '''
    input: mean - mean vector of the multivariate normal distribution, OR a
               single vct3, OR a List[vct3] (one marker/point per entry)
           cov - covariance matrix of the multivariate normal distribution;
               if mean is a List[vct3] of length N and cov is 3x3, the same
               3x3 covariance is applied i.i.d. to each point (block-diagonal
               3N x 3N); a 3N x 3N cov is also accepted directly
           n - number of samples to generate (default is 1)
    output: samples - if mean was a single vct3: a vct3 (n=1) or List[vct3]
               (n>1); if mean was a List[vct3]: a List[vct3] (n=1) or
               List[List[vct3]] (n>1); otherwise, the original ndarray
               behavior of np.random.multivariate_normal
    '''
    single = isinstance(mean, vct3)
    mean_list = [mean] if single else mean

    if isinstance(mean_list, (list, tuple)) and len(mean_list) > 0 and isinstance(mean_list[0], vct3):
        n_points = len(mean_list)
        flat_mean = np.concatenate([m.vec.ravel() for m in mean_list])
        cov_arr = np.asarray(cov, dtype=np.float64)
        if cov_arr.shape == (3, 3) and n_points > 1:
            cov_arr = np.kron(np.eye(n_points), cov_arr)
        samples = np.random.multivariate_normal(flat_mean, cov_arr, n)  # (n, 3*n_points)
        out = [[vct3(row[3 * i:3 * i + 3]) for i in range(n_points)] for row in samples]
        if single:
            out = [row[0] for row in out]
        if n == 1:
            return out[0]
        return out

    return np.random.multivariate_normal(mean, cov, n)


def sample_in_ball(rng: np.random.Generator, r_max: float, dim: int = 3, r_min_frac: float = 0.0) -> np.ndarray:
    '''
    input: rng - numpy Generator
           r_max - radius bound B of the ||x|| <= B spec being sampled
           dim - dimension of the vector
           r_min_frac - lower bound on the norm as a fraction of r_max, so the
               sampled "truth" is never accidentally (near) zero
    output: a random vector with uniformly random direction and norm uniform
            in [r_min_frac * r_max, r_max]
    '''
    direction = rng.normal(size=dim)
    direction /= np.linalg.norm(direction)
    return direction * rng.uniform(r_min_frac * r_max, r_max)


def sample_perpendicular(rng: np.random.Generator, axis: vct3, r_max: float, r_min_frac: float = 0.0) -> np.ndarray:
    '''
    input: rng - numpy Generator
           axis - the vector the sample must be perpendicular to
           r_max, r_min_frac - as in sample_in_ball
    output: a random length-3 vector perpendicular to axis (2 degrees of freedom)
    '''
    a = axis.vec.ravel() / axis.norm()
    v = rng.normal(size=3)
    v -= a * float(a @ v)
    v /= np.linalg.norm(v)
    return v * rng.uniform(r_min_frac * r_max, r_max)
