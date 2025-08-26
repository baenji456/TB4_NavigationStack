# mdn_prediction/mdn_prediction/mdn_sampler.py
import math
from typing import Tuple, List

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

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
        self.declare_parameter('topic_out_single', '/predictions/costmap_t05s')    # Single

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


    def _make_grid_msg(self, grid_prob: np.ndarray, frame_id: str) -> OccupancyGrid:
        # Wahrscheinlichkeit -> 0..100 (int8)
        data = np.clip(np.round(grid_prob * 100.0), 0, 100).astype(np.int8).ravel(order='C')

        og = OccupancyGrid()
        og.header = _now_header(self, frame_id)

        info = MapMetaData()
        info.resolution = float(self.res)
        info.width = self.width
        info.height = self.height
        info.origin.position.x = float(self.x_min)
        info.origin.position.y = float(self.y_min)
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0
        og.info = info

        og.data = data.tolist()
        return og

    # ---- Index finden (per h oder per Zeit) ----
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
            # Zeiten aus msg.horizons (wenn vorhanden), sonst aus msg.dt
            times: List[float] = []
            if len(msg.horizons) == H:
                times = [h.sec + 1e-9 * h.nanosec for h in msg.horizons]
            else:
                dt = float(msg.dt) if msg.dt > 0 else 0.0
                if dt <= 0.0:
                    # fallback: gleichmäßige Schritte 1..H
                    times = [float(k + 1) for k in range(H)]
                else:
                    times = [dt * float(k + 1) for k in range(H)]
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
            grid_prob = self._hist_from_mixture(msg.mixtures[idx])
            og = self._make_grid_msg(grid_prob, frame_id)
            self.pub_single.publish(og)
            return

        # Standard: alle Mixturen als Array
        grids = []
        for mix in msg.mixtures:
            grid_prob = self._hist_from_mixture(mix)
            og = self._make_grid_msg(grid_prob, frame_id)
            grids.append(og)

        out = OccupancyGridArray()
        out.header = _now_header(self, frame_id)
        out.grids = grids
        self.pub_array.publish(out)


def main():
    rclpy.init()
    node = MDNSamplerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()