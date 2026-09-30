function results = schmidt_consider_self_test()
%SCHMIDT_CONSIDER_SELF_TEST Run the T0 and T5 regression checks.

rng_state = rng;
cleanup = onCleanup(@() rng(rng_state));
rng(17, 'twister');
state_count = 15;
beacon_count = 3;
A = randn(state_count);
P = A*A'+eye(state_count);
x = randn(state_count, 1);
Hx = randn(2, state_count);
Hb = zeros(2, 2*beacon_count);
Hb(1, 1:2) = [-0.6, 0.8];
R = diag([36, 0.16]);
z = randn(2, 1);

% T0: zero consider covariance must exactly reduce to the ordinary update.
kf0.RANK = state_count;
kf0.P = P;
kf0.x = x;
consider0.Pxb = zeros(state_count, 2*beacon_count);
consider0.Pbb = zeros(2*beacon_count);
[kf_t0, ~] = schmidt_consider_update(kf0, consider0, Hx, Hb, z, R);
S0 = Hx*P*Hx'+R;
K0 = P*Hx'/S0;
x0 = x+K0*(z-Hx*x);
P0 = 0.5*((P-K0*S0*K0')+(P-K0*S0*K0')');
results.t0_state_max_abs = max(abs(kf_t0.x-x0));
results.t0_covariance_max_abs = max(abs(kf_t0.P-P0), [], 'all');

% T5: with Pxb=0, the first update must equal R_eff processing.
consider5 = consider0;
consider5.Pbb(1:2, 1:2) = diag([25, 49]);
[kf_t5, consider_t5] = schmidt_consider_update( ...
    kf0, consider5, Hx, Hb, z, R);
R_effective = R+Hb*consider5.Pbb*Hb';
S5 = Hx*P*Hx'+R_effective;
K5 = P*Hx'/S5;
x5 = x+K5*(z-Hx*x);
P5 = 0.5*((P-K5*S5*K5')+(P-K5*S5*K5')');
results.t5_state_max_abs = max(abs(kf_t5.x-x5));
results.t5_covariance_max_abs = max(abs(kf_t5.P-P5), [], 'all');
results.t5_pxb_norm_after = norm(consider_t5.Pxb, 'fro');
results.t5_pbb_change_norm = norm(consider_t5.Pbb-consider5.Pbb, 'fro');

% T6-T8: verify accept, soft downweight, and hard reject decisions.
robust_options = struct( ...
    'enabled', true, 'enable_rejection', true, ...
    'soft_threshold', 3.841, 'reject_threshold', 10.828, ...
    'minimum_weight', 0.10);
decision_accept = nis_robust_range_policy(1, 4, 36, robust_options);
decision_downweight = nis_robust_range_policy(20, 4, 36, robust_options);
decision_reject = nis_robust_range_policy(30, 4, 36, robust_options);
results.t6_accept_action = decision_accept.action_code;
results.t6_accept_weight = decision_accept.information_weight;
results.t7_downweight_action = decision_downweight.action_code;
results.t7_effective_nis = decision_downweight.effective_nis;
results.t8_reject_action = decision_reject.action_code;
results.t8_reject_weight = decision_reject.information_weight;
downweight_only_options = robust_options;
downweight_only_options.enable_rejection = false;
decision_capped = nis_robust_range_policy(100, 4, 36, ...
    downweight_only_options);
results.t9_capped_action = decision_capped.action_code;
results.t9_capped_weight = decision_capped.information_weight;

tolerance = 1e-10;
results.passed = results.t0_state_max_abs < tolerance && ...
    results.t0_covariance_max_abs < tolerance && ...
    results.t5_state_max_abs < tolerance && ...
    results.t5_covariance_max_abs < tolerance && ...
    results.t5_pxb_norm_after > tolerance && ...
    results.t5_pbb_change_norm < tolerance && ...
    results.t6_accept_action == 0 && ...
    abs(results.t6_accept_weight-1) < tolerance && ...
    results.t7_downweight_action == 1 && ...
    abs(results.t7_effective_nis-robust_options.soft_threshold) < tolerance && ...
    results.t8_reject_action == 2 && ...
    results.t8_reject_weight == 0 && ...
    results.t9_capped_action == 1 && ...
    abs(results.t9_capped_weight-robust_options.minimum_weight) < tolerance;
if ~results.passed
    error('Schmidt/Consider T0/T5回归检查失败。');
end
clear cleanup;
end
