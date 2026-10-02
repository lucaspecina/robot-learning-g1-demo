import numpy as np

from toolkit.eval.offline import ChunkPrediction, error_by_horizon, error_table


def _prediction(offset: float) -> ChunkPrediction:
    target = np.zeros((4, 2))
    return ChunkPrediction(
        episode_index=0, start=1, predicted=target + offset, target=target,
        hold=target + 1.0, last_action=target + 2.0,
    )


def test_error_table_reports_policy_and_baselines_per_joint():
    table = error_table([_prediction(0.5)], ["a", "b"])
    assert list(table.index) == ["a", "b"]
    assert np.allclose(table["policy"], 0.5)
    assert np.allclose(table["hold"], 1.0)
    assert np.allclose(table["last_action"], 2.0)


def test_error_by_horizon_has_one_row_per_future_step():
    assert len(error_by_horizon([_prediction(0.1), _prediction(0.3)])) == 4
