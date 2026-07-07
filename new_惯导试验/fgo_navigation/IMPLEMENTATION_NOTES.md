# Implementation notes

## Why the earlier implementation could perform poorly

The working EKF/RTS model and the earlier FGO used different definitions:

- The EKF position-error state is `[d_lat(rad), d_lon(rad), d_h(m)]`.
- The earlier FGO retraction treated the first three states as local NED metres.
- The EKF propagation contains Earth rotation, transport rate, gravity
  gradient and first-order Gauss-Markov bias dynamics.
- The earlier preintegration used a separate local constant-gravity model.

Even if both implementations run, their Jacobians, covariance units and
feedback directions are not equivalent.

## New formulation

The new graph solves the same first-order error model as the working filter:

\[
x_{k+1} = \Phi_k x_k + w_k,
\qquad w_k\sim\mathcal{N}(0,Q_k).
\]

For depth:

\[
z_{h,k}=h_k-h_k^{meas}=H_hx_k+v_{h,k}.
\]

For horizontal range:

\[
z_{r,k}=\hat r_k-r_k^{meas}=H_rx_k+v_{r,k}.
\]

The graph minimizes the prior, process, depth and range residuals jointly.
The measurement signs and Jacobians are the same as `myHeightUpdate` and
`myRangeUpdate`.

## Deliberate differences from the current RTS script

1. Depth observations are aggregated to 1 Hz rather than inserted 100 times
   per second. This avoids excessive confidence from strongly correlated
   pressure measurements.
2. The graph includes several sparse range observations in one global solve,
   instead of smoothing one seven-minute block from only its endpoint.
3. Acoustic range factors use Huber IRLS.
4. Height correction uses `h_corrected = h_nominal - dx(3)`, matching
   `myErrorFeedback_range`. The supplied RTS output function uses the opposite
   sign for height in its `rad` branch and should be checked separately.
5. Attitude correction is available. Disable it in options for a strict
   comparison with the supplied feedback function.

## Recommended experiment sequence

1. Set `add_simulated_noise=false` and compare nominal residual signs.
2. Set `apply_attitude_correction=false` and compare against current RTS.
3. Enable simulated noise and compare repeated random seeds.
4. Enable attitude correction only after position/velocity results are stable.
5. Inspect range normalized residuals and robust weights before judging RMSE.
