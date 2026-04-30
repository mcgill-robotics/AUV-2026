"""ThrustAllocator class for allocating optimal thrust commands to each thruster.
For details on mathematical formulation, consult the PDF document in the propulsion package.
"""

import numpy as np
import osqp
import scipy.sparse as sp


class ThrustAllocator:
    def __init__(self, thruster_matrix: np.ndarray, max_thrust: np.ndarray, min_thrust: np.ndarray):
        self.T = np.asarray(thruster_matrix, dtype=float)
        self.max_thrust = np.asarray(max_thrust, dtype=float)
        self.min_thrust = np.asarray(min_thrust, dtype=float)

        if self.T.shape != (6, 8):
            raise ValueError(f"Expected thruster_matrix shape (6, 8), got {self.T.shape}")

        if self.max_thrust.shape != (8,):
            raise ValueError(f"Expected max_thrust shape (8,), got {self.max_thrust.shape}")

        if self.min_thrust.shape != (8,):
            raise ValueError(f"Expected min_thrust shape (8,), got {self.min_thrust.shape}")

        if np.any(self.min_thrust > self.max_thrust):
            raise ValueError("Each min_thrust value must be <= corresponding max_thrust value")

        self.T_inv = np.linalg.pinv(self.T)

        # -- LP Solver setup
        self.P_lp = sp.csc_matrix((9, 9))

        self.q_lp = np.zeros(9)
        self.q_lp[8] = -1.0  # Maximize alpha (minimize -alpha)

        self.scaled_rows = np.array([0, 1, 3, 4, 5])

        tau_placeholder = -np.ones((6, 1))
        tau_placeholder[2, 0] = 0.0  # Do not scale HEAVE

        self.A_lp = sp.bmat([
            [sp.csc_matrix(self.T), sp.csc_matrix(tau_placeholder)],
            [sp.eye(8, format="csc"), sp.csc_matrix((8, 1))],
            [sp.csc_matrix((1, 8)), sp.csc_matrix([[1.0]])],
        ], format="csc")

        self.l_lp = np.concatenate([
            np.zeros(6),
            self.min_thrust,
            [0.0],
        ])

        self.u_lp = np.concatenate([
            np.zeros(6),
            self.max_thrust,
            [1.0],
        ])

        alpha_col = 8
        start = self.A_lp.indptr[alpha_col]
        end = self.A_lp.indptr[alpha_col + 1]
        rows = self.A_lp.indices[start:end]

        self.alpha_col_data_indices = []
        for row in self.scaled_rows:
            matches = np.where(rows == row)[0]
            if len(matches) != 1:
                raise RuntimeError(f"Could not find alpha-column entry for row {row}")
            self.alpha_col_data_indices.append(start + matches[0])

        self.alpha_col_data_indices = np.array(self.alpha_col_data_indices)

        self.lp_solver = osqp.OSQP()
        self.lp_solver.setup(
            P=self.P_lp,
            q=self.q_lp,
            A=self.A_lp,
            l=self.l_lp,
            u=self.u_lp,
            verbose=False,
            warm_starting=True,
        )

        # -- QP Solver setup
        self.P_qp = sp.eye(8, format="csc")
        self.q_qp = np.zeros(8)

        self.A_qp = sp.vstack([
            sp.csc_matrix(self.T),
            sp.eye(8, format="csc"),
        ], format="csc")

        self.l_qp = np.concatenate([
            np.zeros(6),
            self.min_thrust,
        ])

        self.u_qp = np.concatenate([
            np.zeros(6),
            self.max_thrust,
        ])

        self.qp_solver = osqp.OSQP()
        self.qp_solver.setup(
            P=self.P_qp,
            q=self.q_qp,
            A=self.A_qp,
            l=self.l_qp,
            u=self.u_qp,
            verbose=False,
            warm_starting=True,
        )

    def maximize_alpha(self, desired_wrench: np.ndarray) -> float:
        desired_wrench = np.asarray(desired_wrench, dtype=float)

        A_data = self.A_lp.data.copy()
        A_data[self.alpha_col_data_indices] = -desired_wrench[self.scaled_rows]

        l = self.l_lp.copy()
        u = self.u_lp.copy()

        l[2] = desired_wrench[2]
        u[2] = desired_wrench[2]

        self.lp_solver.update(Ax=A_data, l=l, u=u)
        results = self.lp_solver.solve()

        if results.info.status_val != osqp.constant("OSQP_SOLVED"):
            raise RuntimeError(f"LP solver failed with status: {results.info.status}")

        alpha = float(results.x[8])
        return alpha

    def minimize_thrust(self, target_wrench: np.ndarray):
        target_wrench = np.asarray(target_wrench, dtype=float)

        thrust_commands = self.T_inv @ target_wrench

        bound_respected = (
            np.all(thrust_commands >= self.min_thrust)
            and np.all(thrust_commands <= self.max_thrust)
        )

        if bound_respected:
            method = "pseudo-inverse"
            return thrust_commands, method

        self.qp_solver.update(
            l=np.concatenate([target_wrench, self.min_thrust]),
            u=np.concatenate([target_wrench, self.max_thrust]),
        )

        results = self.qp_solver.solve()

        if results.info.status_val != osqp.constant("OSQP_SOLVED"):
            raise RuntimeError(f"QP solver failed with status: {results.info.status}")

        method = "QP"
        return results.x[:8], method

    def allocate_thrust(self, desired_wrench: np.ndarray):
        desired_wrench = np.asarray(desired_wrench, dtype=float)

        if desired_wrench.shape != (6,):
            raise ValueError(f"Expected desired_wrench shape (6,), got {desired_wrench.shape}")

        if np.linalg.norm(desired_wrench) < 1e-3:
            return np.zeros(8), 0.0, "zero"

        alpha = self.maximize_alpha(desired_wrench)

        target_wrench = alpha * desired_wrench
        target_wrench[2] = desired_wrench[2]  # Preserve HEAVE

        thrust_commands, method = self.minimize_thrust(target_wrench)

        return thrust_commands, alpha, method