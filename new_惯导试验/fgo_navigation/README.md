# RTS-consistent error-state factor graph

This project implements batch factor-graph smoothing using the same 15-state
error model as the working EKF/RTS project.

## State convention

The graph variable at each node is

```text
[d_lat(rad), d_lon(rad), d_h(m),
 d_vN, d_vE, d_vD,
 d_att(3),
 d_bg(3), d_ba(3)]
```

The corrected navigation state follows `myErrorFeedback_range`:

```matlab
pos = pos - dx(1:3);
vel = vel - dx(4:6);
gyrbias = gyrbias + dx(10:12);
accbias = accbias + dx(13:15);
```

Attitude correction is disabled by default to match the current feedback
function, where it is commented out. It can be enabled after the position and
velocity graph has been validated.

## Graph construction

- IMU: 100 Hz mechanization. `myInsPropagate_15state` is called at every IMU
  sample and its `phi` and process covariance are accumulated between nodes.
- Graph nodes: 1 Hz by default, with an additional node forced at every range
  timestamp.
- Depth: 100 Hz samples are robustly aggregated into one median observation
  per graph node. The standard deviation is not divided by `sqrt(N)` by
  default, avoiding unrealistic information when pressure samples are
  correlated.
- Range: one factor every 420 seconds. The default `legacy_rows` selection
  reproduces the working RTS experiment. `timestamp` mode is also available.
- Solver: sparse batch weighted least squares with Huber IRLS for acoustic
  range outliers.

## Required existing functions

Place this folder inside the existing KF-GINS MATLAB project, or add the
folders containing these functions to the MATLAB path:

```text
Param, config_1, myInitialize_15state, InsMech,
myInsPropagate_15state, getRmRn, getGravity,
rotvec2quat, quatProd, quatNormalized, quat2dcm, dcm2euler
```

## Run

```matlab
addpath(genpath('path/to/KF-GINS-Matlab'));
addpath('path/to/fgo_rts_consistent');

result = fgo_experiment_rts_consistent( ...
    'F:/2_Data/惯导试验/实验数据/All_data/input6', ...
    'D:/Github/KF-GINS-Matlab/new_惯导试验/fgo_rts_consistent/output6');
```

Options can be overridden with a struct:

```matlab
opts = fgo_default_options();
opts.node_interval_sec = 1;
opts.range_selection_mode = 'legacy_rows';
opts.depth_source = 'truth'; % use 'file' for cfg.heightfilepath
opts.use_depth = true;
opts.use_range = true;
opts.apply_attitude_correction = false; % exact feedback comparison
opts.add_simulated_noise = true;

result = fgo_experiment_rts_consistent(in_dir, output_dir, opts);
```

`legacy_rows` reproduces the reference experiment and assumes each raw range
file is 1 Hz. For real timestamped measurements, use `timestamp`.

`depth_source='truth'` reproduces the supplied simulation, which constructs
noisy depth from the truth file. Use `depth_source='file'` for the real depth
sensor file and normally set `add_simulated_noise=false`.

For an ablation comparison:

```matlab
run_fgo_ablation(in_dir, output_root, opts);
```

This produces INS, INS-depth, INS-range, and fully fused trajectories.

## Output

- `FGO-Keyframes-rad.nav`: corrected graph-node trajectory.
- `FGO-FullRate-rad.nav`: corrected 100 Hz trajectory using interpolated
  smoothed error states.
- `FGO-result.mat`: graph, residual, state correction and diagnostics.
- `FGO-diagnostics.png`: normalized residuals and correction magnitudes.

## Important interpretation

This is a first-order error-state factor graph. It is deliberately close to
the verified EKF/RTS model and is expected to be more reliable than combining
that project with a separate local-frame IMU preintegration implementation.
For very large initial errors, add outer relinearization after confirming the
single-pass graph behaves correctly.
