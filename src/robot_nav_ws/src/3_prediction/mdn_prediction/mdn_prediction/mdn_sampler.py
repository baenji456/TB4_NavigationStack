# mdn_prediction/mdn_prediction/mdn_sampler.py
import math
from typing import Tuple, List
import matplotlib.pyplot as plt

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from rclpy.duration import Duration

from prediction_msgs.msg import Mixture2DSequence, Mixture2D, Gaussian2D, OccupancyGridArray

from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

# "Latched" QoS: 1 letztes Sample, zuverlässig, transient_local
qos_latched = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _now_header(node: Node, frame_id: str) -> Header:
    h = Header()
    h.stamp = node.get_clock().now().to_msg()
    h.frame_id = frame_id
    return h


def _cov_from_flat(sigma4) -> np.ndarray:
    # sigma4 = [xx, xy, yx, yy] (row-major)
    S = np.array([[sigma4[0], sigma4[1]],
                  [sigma4[2], sigma4[3]]], dtype=float)
    eps = 1e-10
    w = np.linalg.eigvalsh(S)
    if w[0] < eps:
        S = S + np.eye(2) * (eps - w[0])
    return S


def _sample_mixture_numpy(mix: Mixture2D, n: int, rng: np.random.Generator) -> np.ndarray:
    """Ziehe n Samples aus einer 2D-Gauss-Mixture (NumPy, vektorisiert)."""
    weights = np.asarray(mix.weights, dtype=float)
    weights = np.clip(weights, 1e-12, None)
    weights = weights / weights.sum()
    M = len(weights)

    comp_ids = rng.choice(M, size=n, p=weights)
    counts = np.bincount(comp_ids, minlength=M)

    samples = np.empty((n, 2), dtype=float)
    start = 0
    for j, nj in enumerate(counts):
        if nj == 0:
            continue
        mu = np.array(mix.components[j].mu, dtype=float)  # (2,)
        S = _cov_from_flat(mix.components[j].sigma)       # (2,2)
        xs = rng.multivariate_normal(mean=mu, cov=S, size=nj, check_valid='ignore')
        end = start + nj
        samples[start:end, :] = xs
        start = end

    rng.shuffle(samples)
    return samples


def _sample_mixture_torch(mix: Mixture2D, n: int):
    """Alternative auf Torch (falls verfügbar)."""
    import torch
    weights = torch.tensor(mix.weights, dtype=torch.double)
    weights = torch.clamp(weights, 1e-12)
    weights = weights / weights.sum()
    M = weights.numel()

    comp_ids = torch.multinomial(weights, num_samples=n, replacement=True)  # (n,)
    unique, counts = comp_ids.unique(return_counts=True)

    samples = torch.empty(n, 2, dtype=torch.double)
    start = 0
    for j, nj in zip(unique.tolist(), counts.tolist()):
        if nj == 0:
            continue
        mu = torch.tensor(mix.components[j].mu, dtype=torch.double)          # (2,)
        s = mix.components[j].sigma
        S = torch.tensor([[s[0], s[1]], [s[2], s[3]]], dtype=torch.double)   # (2,2)
        eps = 1e-10
        eigvals = torch.linalg.eigvalsh(S)
        if eigvals[0] < eps:
            S = S + torch.eye(2, dtype=torch.double) * (eps - eigvals[0])
        L = torch.linalg.cholesky(S)
        z = torch.randn(nj, 2, dtype=torch.double)
        xs = mu + z @ L.T
        end = start + nj
        samples[start:end, :] = xs
        start = end

    perm = torch.randperm(n, dtype=torch.long)
    samples = samples[perm]
    return samples.numpy()

def _mixture_pdf(mix: Mixture2D, X: np.ndarray) -> np.ndarray:
    """
    Dichte der bivariaten Gauss-Mixture an Punkten X (N,2).
    """
    weights = np.asarray(mix.weights, dtype=float)
    weights = np.clip(weights, 1e-12, None)
    weights = weights / weights.sum()
    N = X.shape[0]
    dens = np.zeros(N, dtype=float)

    two_pi = 2.0 * math.pi
    for w, comp in zip(weights, mix.components):
        mu = np.asarray(comp.mu, dtype=float)          # (2,)
        S  = _cov_from_flat(comp.sigma)                # (2,2)
        # Vorberechnungen
        detS = float(np.linalg.det(S))
        if detS <= 0.0:
            # numerische Stabilisierung (sollte selten passieren, _cov_from_flat schiebt schon)
            S = S + np.eye(2) * 1e-10
            detS = float(np.linalg.det(S))
        invS = np.linalg.inv(S)
        norm = w / (two_pi * math.sqrt(detS))
        DX = X - mu                                    # (N,2)
        Q  = np.einsum('ni,ij,nj->n', DX, invS, DX)    # Quadratische Form
        dens += norm * np.exp(-0.5 * Q)
    return dens


class MDNSamplerNode(Node):
    """
    Subscribed:  /prediction/mixtures  (prediction_msgs/Mixture2DSequence)

    output_mode = 'array':
        Published: topic_out (prediction_msgs/OccupancyGridArray)
    output_mode = 'single':
        Published: topic_out_single (nav_msgs/OccupancyGrid)
        -> Auswahl über output_h (Index) oder output_t (Sekunden; nächster Horizont)
    """

    def __init__(self):
        super().__init__('mdn_sampler')

        # ---- Parameter ----
        self.declare_parameter('topic_in', '/predictions/mixtures')
        self.declare_parameter('topic_out', '/predictions/costmaps')          # Array
        self.declare_parameter('topic_out_single', '/predictions/costmap_t1_5s')    # Single

        # Arbeitsbereich (m) & Auflösung
        self.declare_parameter('x_min', -10.0)
        self.declare_parameter('x_max',  10.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max',  10.0)
        self.declare_parameter('resolution', 0.25)

        # Sampling
        self.declare_parameter('samples_per_step', 5000)
        self.declare_parameter('seed', 0)                # 0 => deterministisch; <0 => kein Seed
        self.declare_parameter('backend', 'numpy')       # 'numpy' oder 'torch'

        # Ausgabe-Steuerung
        self.declare_parameter('output_mode', 'array')   # 'array' | 'single'
        self.declare_parameter('output_h', -1)           # Index [0..H-1], -1=per Zeit
        self.declare_parameter('output_t', -1.0)         # Zielzeit [s], -1=deaktiviert

        # Perzentil-/HPD-Output (Paper-CLs)
        self.declare_parameter('percentile_mode', False)   # False = altes Verhalten
        self.declare_parameter('percentile_p68', 0.68)
        self.declare_parameter('percentile_p85', 0.85)
        self.declare_parameter('percentile_p95', 0.95)
        # Schreibwerte in die Costmap (0..100)
        self.declare_parameter('percentile_value_else', 0)
        self.declare_parameter('percentile_value_68', 50)
        self.declare_parameter('percentile_value_85', 75)
        self.declare_parameter('percentile_value_95', 100)

        topic_in   = str(self.get_parameter('topic_in').value)
        topic_outA = str(self.get_parameter('topic_out').value)
        topic_outS = str(self.get_parameter('topic_out_single').value)

        # RNG
        seed = int(self.get_parameter('seed').value)
        if seed >= 0:
            self.rng = np.random.default_rng(seed)
        else:
            self.rng = np.random.default_rng()

        # Backend
        self.backend = str(self.get_parameter('backend').value).lower()
        self.torch_available = False
        if self.backend == 'torch':
            try:
                import torch  # noqa: F401
                self.torch_available = True
            except Exception:
                self.get_logger().warn("Torch nicht verfügbar – falle auf NumPy zurück.")
                self.backend = 'numpy'

        # Grid-Metadaten
        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.res   = float(self.get_parameter('resolution').value)

        self.width  = int(math.ceil((self.x_max - self.x_min) / self.res))
        self.height = int(math.ceil((self.y_max - self.y_min) / self.res))

        # Publisher / Subscriber
        self.sub = self.create_subscription(Mixture2DSequence, topic_in, self.on_mixtures, 10)
        # <<< HIER latched Publisher setzen >>>
        self.pub_array  = self.create_publisher(OccupancyGridArray, topic_outA, qos_latched)
        self.pub_single = self.create_publisher(OccupancyGrid,        topic_outS, qos_latched)

        self.get_logger().info(
            f"mdn_sampler started | in={topic_in} | out(array)={topic_outA}, out(single)={topic_outS} | "
            f"grid: {self.width}x{self.height} @ {self.res} m"
        )

        # --- Plot Setup ---
        self.fig, self.axs = plt.subplots(1, 4, figsize=(12, 3))
        self.imgs = None
        plt.ion()
        plt.show()

    # ---- Sampling einer einzelnen Mixture zu Histogramm ----
    def _hist_from_mixture(self, mix: Mixture2D) -> np.ndarray:
        S = int(self.get_parameter('samples_per_step').value)

        if self.backend == 'torch' and self.torch_available:
            samples = _sample_mixture_torch(mix, S)
        else:
            samples = _sample_mixture_numpy(mix, S, self.rng)  # (S,2)

        # Bin in Zellen (vektorisiert)
        ix = np.floor((samples[:, 0] - self.x_min) / self.res).astype(np.int64)
        iy = np.floor((samples[:, 1] - self.y_min) / self.res).astype(np.int64)

        mask = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        if not np.any(mask):
            return np.zeros((self.height, self.width), dtype=np.float64)

        ix = ix[mask]; iy = iy[mask]
        flat_idx = iy * self.width + ix  # row-major
        counts = np.bincount(flat_idx, minlength=self.width * self.height).astype(np.float64)
        grid = counts.reshape(self.height, self.width)

        # Normieren auf [0..1] als Wahrscheinlichkeit (Summe = 1 über Grid)
        #s = grid.sum()
        #if s > 0:
        #    grid /= s 
        #    return grid
        # Normieren auf [0..1] (Minimum -> 0, Maximum -> 1)
        min_val = grid.min()
        max_val = grid.max()
        if max_val > min_val:   # Vermeidet Division durch 0
            grid = (grid - min_val) / (max_val - min_val)
        return grid

    def _percentile_grid_from_mixture(self, mix: Mixture2D) -> np.ndarray:
        """
        Liefert ein Grid (height,width) mit Werten in [0..1], die nachher *100 geschrieben werden.
        Die Werte sind NICHT Wahrscheinlichkeiten, sondern die konfigurierten Costmap-Werte
        (z.B. 0.50, 0.75, 1.00 für 68/85/95%) – überall sonst 0.0.
        """
        # Parameter holen
        p68 = float(self.get_parameter('percentile_p68').value)
        p85 = float(self.get_parameter('percentile_p85').value)
        p95 = float(self.get_parameter('percentile_p95').value)

        v_else = float(self.get_parameter('percentile_value_else').value)
        v68    = float(self.get_parameter('percentile_value_68').value)
        v85    = float(self.get_parameter('percentile_value_85').value)
        v95    = float(self.get_parameter('percentile_value_95').value)

        # (a) Samples für CL-Schätzung ziehen (wie beim Histogramm)
        S = int(self.get_parameter('samples_per_step').value)
        if self.backend == 'torch' and self.torch_available:
            samples = _sample_mixture_torch(mix, S)
        else:
            samples = _sample_mixture_numpy(mix, S, self.rng)  # (S,2)

        # Dichten der Samples (für ECDF der Dichte)
        dens_samples = _mixture_pdf(mix, samples)
        dens_sorted = np.sort(dens_samples)  # aufsteigend

        # (b) Zellmittelpunkte erzeugen
        xs = self.x_min + (np.arange(self.width)  + 0.5) * self.res
        ys = self.y_min + (np.arange(self.height) + 0.5) * self.res
        XX, YY = np.meshgrid(xs, ys)                     # (H,W)
        pts = np.column_stack([XX.ravel(order='C'), YY.ravel(order='C')])  # (H*W, 2)

        # (c) Dichte an Zellmitteln
        dens_cells = _mixture_pdf(mix, pts)              # (H*W,)

        # (d) CL-Schätzung 1 - alpha(p) = P(D(Z) >= D(p)) per ECDF
        # Anteil >= d  ==  (S - idx)/S, mit idx = # {dens < d}
        idx = np.searchsorted(dens_sorted, dens_cells, side='left')
        CL = (S - idx).astype(np.float64) / float(S)     # (H*W,) in [0,1]

        # (e) HPD-Bänder: Punkt gehört zur (1-alpha)-Region, wenn CL(p) <= (1-alpha)
        vals = np.full(CL.shape, v_else, dtype=np.float64)
        # 68% (innerste Region) hat kleinste CL-Schwelle:
        mask68 = CL <= p68
        mask85 = (CL <= p85) & ~mask68
        mask95 = (CL <= p95) & ~mask68 & ~mask85
        vals[mask68] = v68
        vals[mask85] = v85
        vals[mask95] = v95

        # in [0..1] normalisieren, weil _make_grid_msg *100 skaliert
        vals = np.clip(vals / 100.0, 0.0, 1.0)
        return vals.reshape(self.height, self.width)



    def _make_grid_msg(self, grid_prob: np.ndarray, frame_id: str, stamp=None) -> OccupancyGrid:
        # Wahrscheinlichkeit -> 0..100 (int8)
        data = np.clip(np.round(grid_prob * 100.0), 0, 100).astype(np.int8).ravel(order='C')

        og = OccupancyGrid()
        if stamp is None:
            og.header = _now_header(self, frame_id)
        else:
            og.header.frame_id = frame_id
            og.header.stamp = stamp

        info = MapMetaData()
        info.resolution = float(self.res)
        info.width = self.width
        info.height = self.height
        info.map_load_time = og.header.stamp
        info.origin.position.x = float(self.x_min)
        info.origin.position.y = float(self.y_min)
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0
        og.info = info

        og.data = data.tolist()
        return og

    # ---- Index finden (per h oder per Zeit) ----
    def _horizon_seconds(self, msg: Mixture2DSequence) -> List[float]:
        """Liefert die Horizonte in Sekunden pro Schritt."""
        H = len(msg.mixtures)
        if len(msg.horizons) == H:
            return [h.sec + 1e-9 * h.nanosec for h in msg.horizons]
        dt = float(msg.dt) if msg.dt > 0 else 0.0
        if dt <= 0.0:
            return [float(k + 1) for k in range(H)]
        else:
            return [dt * float(k + 1) for k in range(H)]

    def _select_index(self, msg: Mixture2DSequence) -> int:
        H = len(msg.mixtures)
        if H == 0:
            return -1

        out_h = int(self.get_parameter('output_h').value)
        if out_h >= 0:
            if out_h >= H:
                self.get_logger().warn(f"output_h={out_h} >= H={H}, klemme auf {H-1}.")
                out_h = H - 1
            return out_h

        out_t = float(self.get_parameter('output_t').value)
        if out_t >= 0.0:
            times = self._horizon_seconds(msg)
            idx = int(np.argmin(np.abs(np.asarray(times) - out_t)))
            return idx

        # Default: letzter Zeitschritt
        return H - 1

    # ---- Callback ----
    def on_mixtures(self, msg: Mixture2DSequence):
        frame_id = msg.header.frame_id if msg.header.frame_id else 'map'
        mode = str(self.get_parameter('output_mode').value).lower()

        if mode == 'single':
            idx = self._select_index(msg)
            if idx < 0 or idx >= len(msg.mixtures):
                self.get_logger().warn("Keine Mixture vorhanden, publiziere nicht.")
                return
            # Vorhersagezeitpunkt ermitteln und in Header stempeln
            times = self._horizon_seconds(msg)
            pred_stamp = (self.get_clock().now() + Duration(seconds=times[idx])).to_msg()
 
            if bool(self.get_parameter('percentile_mode').value):
                grid_val = self._percentile_grid_from_mixture(msg.mixtures[idx])
            else:
                grid_val = self._hist_from_mixture(msg.mixtures[idx])

            og = self._make_grid_msg(grid_val, frame_id, stamp=pred_stamp)
            self.pub_single.publish(og)
            return

        # Standard: alle Mixturen als Array
        grids = []
        grid_probs = []
        times = self._horizon_seconds(msg)
        now = self.get_clock().now()
        for k, mix in enumerate(msg.mixtures):
            if bool(self.get_parameter('percentile_mode').value):
                grid_val = self._percentile_grid_from_mixture(msg.mixtures[k])
            else:
                grid_val = self._hist_from_mixture(msg.mixtures[k])

            pred_stamp = (now + Duration(seconds=times[k])).to_msg()
            og = self._make_grid_msg(grid_val, frame_id, stamp=pred_stamp)
            grids.append(og)
            grid_probs.append(grid_val)

        out = OccupancyGridArray()
        out.header = _now_header(self, frame_id)
        out.grids = grids
        self.pub_array.publish(out)

        # --- Live-Plot aktualisieren ---
        H = len(grid_probs)
        if H >= 4:
            idxs = [0, H // 3, 2 * H // 3, H - 1]
        else:
            idxs = list(range(H))  # falls nur wenige Horizonte

        selected = [grid_probs[i] for i in idxs]
        horizon_times = [times[i] for i in idxs]

        if self.imgs is None:
            self.imgs = []
            for ax, grid, t in zip(self.axs, selected, horizon_times):
                im = ax.imshow(grid, origin='lower', cmap='hot', vmin=0, vmax=1)
                ax.set_title(f"{t:.1f}s")
                self.imgs.append(im)
            plt.tight_layout()
        else:
            for im, grid, ax, t in zip(self.imgs, selected, self.axs, horizon_times):
                im.set_data(grid)
                ax.set_title(f"{t:.1f}s")

        plt.pause(0.001)


def main():
    rclpy.init()
    node = MDNSamplerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()