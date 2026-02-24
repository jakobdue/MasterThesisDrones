import numpy as np
import matplotlib.pyplot as plt

def B3(v: np.ndarray) -> np.ndarray:
    """
    Cubic B-spline basis B^3(v) from the paper:
      2/3 - v^2 + 1/2|v|^3     for |v| < 1
      (1/6)(2 - |v|)^3         for 1 <= |v| < 2
      0                        for 2 <= |v|
    """
    a = np.abs(v)
    out = np.zeros_like(v, dtype=float)

    m1 = a < 1
    out[m1] = (2/3) - a[m1]**2 + 0.5 * a[m1]**3

    m2 = (a >= 1) & (a < 2)
    out[m2] = (1/6) * (2 - a[m2])**3

    return out

def h_eps(z: np.ndarray, eps: float) -> np.ndarray:
    """h_eps(z) = (3/2) * B^3(2z/eps)."""
    return 1.5 * B3(2 * z / eps)

def p_eps(z: np.ndarray, eps: float, n: int) -> np.ndarray:
    """p_eps(z) = h_eps(z) / z^(n-1) for n=2 or 3."""
    if n not in (2, 3):
        raise ValueError("n must be 2 or 3")
    return h_eps(z, eps) / (z ** (n - 1))

def log_barrier(z: np.ndarray, eps: float) -> np.ndarray:
    """
    A common IPC-style log barrier for comparison (not necessarily identical
    to any specific paper's exact definition):
      -log(z/eps) for z in (0, eps], and 0 for z >= eps.
    """
    out = np.zeros_like(z, dtype=float)
    m = z < eps
    out[m] = -np.log(np.maximum(z[m] / eps, 1e-300))
    return out

def main():
    eps = 1.0  # try changing this (e.g., 0.5, 2.0) to see scaling
    zmin = 1e-4 * eps  # avoid z=0 singularity
    z = np.linspace(zmin, eps, 2000)

    # 1) Plot B^3(v)
    v = np.linspace(-3, 3, 2000)
    plt.figure()
    plt.plot(v, B3(v))
    plt.axvline(-2, linestyle="--")
    plt.axvline(2, linestyle="--")
    plt.title(r"Cubic B-spline $B^3(v)$ (support on $|v|<2$)")
    plt.xlabel(r"$v$")
    plt.ylabel(r"$B^3(v)$")
    plt.ylim(bottom=-0.05)
    plt.show()

    # 2) Plot h_eps(z)
    plt.figure()
    plt.plot(z, h_eps(z, eps))
    plt.axvline(eps, linestyle="--")
    plt.title(rf"Cutoff $h_\varepsilon(z)=\frac{{3}}{{2}}B^3(2z/\varepsilon)$ with $\varepsilon={eps}$")
    plt.xlabel(r"$z$")
    plt.ylabel(r"$h_\varepsilon(z)$")
    plt.show()

    # 3) Plot p_eps(z) for n=2 and n=3
    plt.figure()
    plt.plot(z, p_eps(z, eps, n=2), label=r"$n=2:\; p_\varepsilon(z)=h_\varepsilon(z)/z$")
    plt.plot(z, p_eps(z, eps, n=3), label=r"$n=3:\; p_\varepsilon(z)=h_\varepsilon(z)/z^2$")
    plt.axvline(eps, linestyle="--")
    plt.title(rf"Barrier $p_\varepsilon(z)$ on $(0,\varepsilon]$ (diverges as $z\to 0$), $\varepsilon={eps}$")
    plt.xlabel(r"$z$")
    plt.ylabel(r"$p_\varepsilon(z)$")
    plt.yscale("log")  # makes divergence easier to see
    plt.legend()
    plt.show()

    # 4) Optional: compare p_eps with a log barrier shape
    plt.figure()
    plt.plot(z, p_eps(z, eps, n=2), label=r"Spline barrier (n=2)")
    plt.plot(z, log_barrier(z, eps), label=r"Log barrier (shape comparison)")
    plt.axvline(eps, linestyle="--")
    plt.title(rf"Spline barrier vs. log barrier (both cut off at $\varepsilon={eps}$)")
    plt.xlabel(r"$z$")
    plt.ylabel(r"barrier value")
    plt.yscale("log")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()