# mdn_prediction/dummy_mdn_prediction_node.py
import math
from typing import List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray

from prediction_msgs.msg import Gaussian2D, Mixture2D, Mixture2DSequence


def make_pd_cov(sigx: float, sigy: float, rho: float) -> np.ndarray:
    """Erzeuge 2x2-Kovarianzmatrix aus sigmas und Korrelation; stelle PSD sicher."""
    cov = np.array([
        [sigx * sigx, rho * sigx * sigy],
        [rho * sigx * sigy, sigy * sigy]
    ], dtype=float)
    # numerische Stabilisierung -> kleinsten Eigenwert >= eps schieben
    eps = 1e-10
    lam_min = np.linalg.eigvalsh(cov)[0]
    if lam_min < eps:
        cov += np.eye(2) * (eps - lam_min)
    return cov  # symmetrisch & PSD per Definition. :contentReference[oaicite:4]{index=4}


class DummyMDNPredictionNode(Node):
    """
    Liest /tracked_object/poses (PoseArray) und erzeugt für H Schritte
    Mixture-of-Gaussians (3 Komponenten) gemäß Dummy-Schema.
    """

    def __init__(self):
        super().__init__('dummy_mdn_prediction_node')

        # -------- Parameter --------
        self.declare_parameter('dt', 0.1)                 # [s] Zeitschritt der Vorhersage
        self.declare_parameter('H', 30)                   # Horizontlänge
        self.declare_parameter('forward_speed', 1.0)      # [m/s] entlang Tangente
        self.declare_parameter('topic_in', '/tracked_object/poses')
        self.declare_parameter('topic_out', '/predictions/mixtures')

        # Querdrift-Faktoren (wie in deinem Beispiel)
        self.declare_parameter('drift_base', 0.55)        # für "geradeaus"
        self.declare_parameter('drift_side', 0.40)        # für links/rechts

        # Kovarianz-Basis & Wachstum über h
        self.declare_parameter('sigx0', 0.20)
        self.declare_parameter('sigy0', 0.05)
        self.declare_parameter('dsigx', 0.05)             # additiv pro Schritt
        self.declare_parameter('dsigy', 0.06)
        self.declare_parameter('rho0',  0.2)
        self.declare_parameter('rho1', -0.3)
        self.declare_parameter('rho2',  0.1)

        # Gewichtsdynamik
        self.declare_parameter('w0_base', 0.45)
        self.declare_parameter('w1_base', 0.30)
        self.declare_parameter('dw0', -0.01)              # pro Schritt
        self.declare_parameter('dw1',  0.005)

        # -------- Topics --------
        self.dt = float(self.get_parameter('dt').value)
        self.H = int(self.get_parameter('H').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)

        topic_in = str(self.get_parameter('topic_in').value)
        topic_out = str(self.get_parameter('topic_out').value)

        self.sub = self.create_subscription(PoseArray, topic_in, self.on_poses, 10)
        self.pub = self.create_publisher(Mixture2DSequence, topic_out, 10)

        self.get_logger().info(
            f"dummy_mdn_prediction_node started: H={self.H}, dt={self.dt:.3f}s, "
            f"forward_speed={self.forward_speed:.2f} m/s, in={topic_in}, out={topic_out}"
        )

    # ---------- Helper: Richtung & Orthonormalbasis ----------
    @staticmethod
    def _unit(vec: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(vec)
        return vec / (n + 1e-12)

    def _tangent_normal_from_history(self, poses: List[np.ndarray]) -> (np.ndarray, np.ndarray, np.ndarray):
        """
        Ableitung der Tangente u_t (Bewegungsrichtung), Normalen u_n, und p_now aus Pose-Liste.
        Nimmt die letzten zwei unterschiedlichen Punkte; fällt zurück auf (1,0), wenn stationär.
        """
        p_now = poses[-1]
        # suche rückwärts einen unterschiedlichen Punkt
        p_prev = None
        for i in range(len(poses) - 2, -1, -1):
            if np.linalg.norm(poses[i] - p_now) > 1e-9:
                p_prev = poses[i]
                break
        if p_prev is None:
            u_t = np.array([1.0, 0.0], dtype=float)
        else:
            u_t = self._unit(p_now - p_prev)

        u_n = np.array([-u_t[1], u_t[0]], dtype=float)  # 90° links
        v_hist = u_t * self.forward_speed               # konstante Vorwärtsgeschwindigkeit
        return v_hist, u_t, u_n, p_now

    # ---------- Callback ----------
    def on_poses(self, msg: PoseArray):
        # PoseArray: Header + Liste von Posen. :contentReference[oaicite:5]{index=5}
        if not msg.poses:
            return
        poses_xy = [np.array([p.position.x, p.position.y], dtype=float) for p in msg.poses]
        v_hist, u_t, u_n, p_now = self._tangent_normal_from_history(poses_xy)

        # Erzeuge Sequenz-Nachricht
        out = Mixture2DSequence()
        out.header = msg.header  # übernimmt Zeit/Frame aus PoseArray
        out.dt = float(self.dt)

        # Vorhersage-Schritte 1..H
        for h in range(self.H):
            t = (h + 1) * self.dt
            base = p_now + t * v_hist

            # leichte Drifts quer zur Bewegung (wie in deinem Dummy-Schema)
            drift_base = float(self.get_parameter('drift_base').value)
            drift_side = float(self.get_parameter('drift_side').value)
            mu0 = base + (drift_base * t) * u_n
            mu1 = base + (drift_side * t) * u_n   # "links"
            mu2 = base - (drift_side * t) * u_n   # "rechts"

            # Kovarianzen (wachsend mit h)
            sigx  = float(self.get_parameter('sigx0').value) + h * float(self.get_parameter('dsigx').value)
            sigy  = float(self.get_parameter('sigy0').value) + h * float(self.get_parameter('dsigy').value)
            rho0  = float(self.get_parameter('rho0').value)
            rho1  = float(self.get_parameter('rho1').value)
            rho2  = float(self.get_parameter('rho2').value)

            S0 = make_pd_cov(sigx,        sigy,        rho0)
            S1 = make_pd_cov(sigx * 1.1,  sigy * 1.1,  rho1)
            S2 = make_pd_cov(sigx * 1.1,  sigy * 1.1,  rho2)

            # Gewichte
            w0 = float(self.get_parameter('w0_base').value) + h * float(self.get_parameter('dw0').value)
            w1 = float(self.get_parameter('w1_base').value) + h * float(self.get_parameter('dw1').value)
            w2 = 1.0 - (w0 + w1)
            weights = np.clip(np.array([w0, w1, w2], dtype=float), 1e-4, None)
            weights = weights / weights.sum()

            # In Messages füllen
            mix_msg = Mixture2D()
            mix_msg.weights = list(map(float, weights))

            def to_gauss(mu: np.ndarray, S: np.ndarray) -> Gaussian2D:
                g = Gaussian2D()
                g.mu = [float(mu[0]), float(mu[1])]
                # row-major xx,xy,yx,yy
                g.sigma = [float(S[0, 0]), float(S[0, 1]), float(S[1, 0]), float(S[1, 1])]
                return g

            mix_msg.components = [to_gauss(mu0, S0), to_gauss(mu1, S1), to_gauss(mu2, S2)]
            out.mixtures.append(mix_msg)

            # Relative Zeit als Duration
            dur = Duration()
            sec = int(t)
            dur.sec = sec
            dur.nanosec = int((t - sec) * 1e9)
            out.horizons.append(dur)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = DummyMDNPredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
