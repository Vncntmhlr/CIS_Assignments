"""Offline checks of practice scoring bookkeeping, with synthetic labels."""

import pytest

from experiments.validate_step8 import outlier_metrics


def row(index, used=True, missing=False):
    return {"sample_index": index, "d_ref": None if missing else [0, 0, 0], "used": used}


def test_original_indices_and_missing_observations_are_separate():
    samples = [row(10, False), row(30), row(70, False), row(90),
               row(120, False, missing=True), row(180, False, missing=True)]
    result = outlier_metrics(samples, {10, 90, 120})
    assert (result["tp"], result["fp"], result["fn"], result["tn"]) == (1, 1, 1, 1)
    assert result["flagged_indices"] == [10, 70]
    assert result["missing_indices"] == [120, 180]
    assert result["true_outliers_missing"] == [120]
    assert result["precision"] == result["recall"] == result["f1"] == 0.5


def test_no_positives_has_undefined_precision_and_recall():
    result = outlier_metrics([row(100), row(130)], set())
    assert result["tn"] == 2
    assert result["precision"] is result["recall"] is result["f1"] is None


@pytest.mark.parametrize("samples,truth", [([row(10), row(10)], set()), ([row(10)], {0})])
def test_misaligned_truth_or_duplicate_indices_are_errors(samples, truth):
    with pytest.raises(ValueError):
        outlier_metrics(samples, truth)
