import numpy as np

from toolkit.checks.normalization import feature_gap


def test_identical_stats_have_no_gap():
    stats = {"mean": np.array([0.0, 1.0]), "std": np.array([1.0, 2.0])}
    gap = feature_gap("action", stats, stats)
    assert gap.max_mean_shift == 0.0
    assert gap.min_std_ratio == gap.max_std_ratio == 1.0


def test_mean_shift_is_measured_in_policy_std():
    policy = {"mean": np.array([0.0, 0.0]), "std": np.array([1.0, 0.5])}
    dataset = {"mean": np.array([0.1, 0.5]), "std": np.array([1.0, 0.5])}
    gap = feature_gap("action", dataset, policy)
    assert np.isclose(gap.max_mean_shift, 1.0)
    assert gap.worst_dim == 1


def test_frozen_joint_does_not_divide_by_zero():
    policy = {"mean": np.array([0.0]), "std": np.array([0.0])}
    dataset = {"mean": np.array([0.0]), "std": np.array([0.0])}
    assert feature_gap("action", dataset, policy).max_mean_shift == 0.0
