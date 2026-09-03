# PythonLinearNonlinearControl: control theory guide

## Core abstraction

The repository separates **model -> planner -> controller -> runner**, which is an excellent software architecture for control experiments. The model predicts the plant; the planner generates the desired state trajectory; the controller computes inputs; the runner closes the simulation loop.

## Linear MPC

For

$$
x_{k+1}=Ax_k+Bu_k,
$$

the controller predicts a sequence of states and solves a finite-horizon optimization. A standard tracking objective is

$$
J=\sum_{k=0}^{N-1}\left[(x_k-x_k^{ref})^TQ(x_k-x_k^{ref})+(u_k-u_k^{ref})^TR(u_k-u_k^{ref})\right]+(x_N-x_N^{ref})^TP(x_N-x_N^{ref}).
$$

In `controllers/mpc.py`, `A`, `B`, `Q`, `R` are model and cost matrices; `pred_len` is $N$; `phi_mat` collects powers of $A$; `gamma_mat` and `theta_mat` collect input-to-state effects. The condensed prediction is essentially

$$
X=\Phi x_k+\Theta U.
$$

`G` is the linear term of the quadratic objective and `H` its Hessian. `W` encodes input-rate constraints; `F` encodes absolute input constraints. `np.cumsum` converts a sequence of increments $\Delta u$ into absolute controls:

$$
u_i=u_{i-1}+\Delta u_i.
$$

The warm start `prev_sol` reuses the previous solution, which is a practical receding-horizon optimization technique.

## Why this repository is useful

Unlike a single MPC implementation, the project contains CEM, MPPI, iLQR, DDP, NMPC and constrained NMPC-CGMRES. That lets you compare optimization philosophies:

- **MPC/QP:** deterministic convex optimization for linear dynamics.
- **CEM:** sample control sequences, retain elite samples, refit the sampling distribution.
- **MPPI:** importance-weight sampled trajectories using their costs.
- **iLQR:** locally linearize dynamics and quadratize cost, then use Riccati-style backward/forward passes.
- **DDP:** iLQR plus second-order dynamics information.
- **NMPC:** repeatedly solve a nonlinear optimal-control problem.

The README correctly notes that linear MPC can be applied to a nonlinear environment after linearization; the distinction between *environment* and *model* is fundamental.

## PID and estimator integration

A practical stack is

$$
\text{sensors}\rightarrow\text{EKF}\rightarrow\text{MPC/iLQR}\rightarrow\text{low-level PID}\rightarrow\text{plant}.
$$

The EKF estimates unmeasured states. The optimizer reasons over multiple future steps. PID provides a robust high-rate inner loop.

## Industrial suitability

Use linear MPC for constrained MIMO systems near an operating point: thermal systems, vehicle longitudinal/lateral control, process loops, and servo systems. Use nonlinear MPC/iLQR/DDP when operating over a broad nonlinear envelope, such as aggressive vehicle maneuvers or robotic motion. Sampling methods such as MPPI/CEM are attractive when derivatives are difficult or discontinuities exist, but their computational budget can be much higher.

## Validation

Do not compare algorithms only by visual smoothness. Fix the same plant, reference and disturbance seed and measure tracking RMS, peak error, constraint violations, control effort, solver wall time and worst-case latency. Then run SIL against an independent implementation and HIL against a deterministic real-time plant.
