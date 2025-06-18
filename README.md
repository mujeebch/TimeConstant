# TimeConstant



### CUSUM-Based Stateful Detection

Our operation transition times are passed as input to a cumulative sum (CUSUM) procedure, which we use as a stateful detector. A unique instance of the procedure is used for each combination of actuator and type of operation (ON/OFF). This use of CUSUM enables the detection of changes in the statistical properties of measured transition times. Each transition time is fed to the corresponding detector as a single scalar value as soon as it is measured.

We first define the symbols used in explaining the procedure:

- `a`: Actuator under tracking  
- `p ∈ {ON, OFF}`: Type of actuator operation  
- `i ∈ ℕ₀`: Time step (with 1st iteration at `i = 1`)  
- `k ∈ {+, -}`: Direction of tracked changes  
- `Sₐ,ₚ,ᵢᵏ ∈ ℝ`: CUSUM  
- `Tₐ,ₚᵏ ∈ ℝ`: CUSUM threshold  
- `μₐ,ₚ ∈ ℝ⁺`: Mean transition time  
- `βₐ,ₚ ∈ ℝ⁺`: Bias  
- `tₐ,ₚ,ᵢ ∈ ℝ⁺`: Measured transition time  



### CUSUM Detector Iteration Rule

Our CUSUM detector [1] iterates using the following rule:

```latex
\begin{equation}
\left\{
\begin{array}{ll}
d_{a, p, i}^+ = S_{a, p, i - 1}^+ + t_{a, p, i} - \mu_{a, p} - \beta_{a, p},\\
d_{a, p, i}^- = S_{a, p, i - 1}^- + t_{a, p, i} - \mu_{a, p} + \beta_{a, p},\\
S_{a, p, i}^+ =
\begin{cases}
\max(0, d_{a, p, i}^+),& \text{if } d_{a, p, i}^+ \leq T_{a, p}^+ \\
0 & \text{otherwise}
\end{cases},\\
S_{a, p, i}^- =
\begin{cases}
\min(0, d_{a, p, i}^-),& \text{if } d_{a, p, i}^- \geq T_{a, p}^- \\
0 & \text{otherwise}
\end{cases},\\
\end{array}
\right.
\label{eq:detector_cusum_iteration_rule}
\end{equation}
```

When `d_{a, p, i}^+ > T_{a, p}^+` and/or `d_{a, p, i}^- < T_{a, p}^-`, the CUSUM detector raises an alarm.

The mean transition time `\mu_{a, p}` is derived by taking the average transition time found in a given SWaT dataset for each actuator-operation combination. The bias `\beta_{a, p}` is derived by halving the standard deviation of each such combination's transition times, and facilitates gradual drifting of the CUSUM values to 0 under normal conditions. The threshold `T_{a, p}^k` is derived using binary search such that a fixed maximum false alarm rate (in this case 2%) is reached on the dataset in each direction.

---

### Iteration Example

In this section we provide an example of the results of performing CUSUM detection on ON transition times of the motorized valve MV101. The table below lists the relevant parameters used for this procedure.

```markdown
| Parameter     | Value  |
|--------------|--------|
| μₐ,ₚ         | 17.79  |
| βₐ,ₚ         | 1.12   |
| Tₐ,ₚ⁺        | 6.56   |
| Tₐ,ₚ⁻        | -3.05  |
```

Figures (a)–(b) indicate the transition times recorded for MV101's ON operations, as well as the alarms raised for each direction of change. There are 2 alarms raised for positive CUSUM, and 14 alarms for negative CUSUM. We also illustrate the start of alarm-inducing changes. For each such change, the start is defined as the latest iteration preceding the alarm, where CUSUM was set to 0 as a result of the `max` function for positive CUSUM and the `min` function for negative CUSUM in the equation above.

Figure (c) illustrates CUSUM values throughout each iteration. It is useful to note that, in the equation above, negative values are rounded to 0 when calculating positive direction CUSUM for each iteration, and that CUSUM values are set to 0 in the case of an alarm being raised, both of which account for the zero CUSUM values found throughout most iterations.

---

### State Space System Model

In the following state space matrices, Kalman Filter gain `L`, input/output vectors are shown along with the estimation equation for the case of `n=4` states. This system model captures the process dynamics of Stage 1 of the SWaT process.

```latex
A = \begin{bmatrix}
1.0000 & 0.0008 & -0.0003 & 0.0031 \\
-0.0026 & 0.9782 & 0.1173 & -0.0037 \\
-0.0057 & -0.0614 & 0.7645 & 0.3523 \\
-0.0091 & 0.0030 & -0.0417 & 0.8197
\end{bmatrix}

B = \begin{bmatrix}
0.0000 & 0.0000 & -0.0000 \\
-0.0003 & 0.0001 & 0.0000 \\
0.0009 & -0.0007 & 0.0001 \\
-0.0010 & 0.0004 & 0.0002
\end{bmatrix}

u_k = \begin{bmatrix}
MV-101 \\
P-101 \\
P-102
\end{bmatrix}, \quad
y_k = \begin{bmatrix}
FIT-101 \\
LIT-101
\end{bmatrix}

C = 1.0e+04 * \begin{bmatrix}
0.0018 & -0.0128 & -0.0006 & -0.0001 \\
-2.9695 & -0.0029 & 0.0002 & -0.0028
\end{bmatrix}

L = \begin{bmatrix}
-0.0001 & -0.0000 \\
-0.0073 & -0.0001 \\
-0.0282 & 0.0010 \\
-0.0038 & -0.0020
\end{bmatrix}
```

Estimation equations:

```latex
\begin{equation}
\left\{
\begin{array}{ll}
\hat{x}_{k+1} = A\hat{x}_k + Bu_k + L(y_k - C\hat{x}_k), \\
\hat{y}_{k} = C\hat{x}_{k}
\end{array}
\right.
\end{equation}
```

