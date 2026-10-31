"""The focused task can rebuild its only processed input from original labels."""
import json
import shutil

import numpy as np
import pytest

from adaptive_control_project.retarget_pipeline.data_convert import convert_task_sequence
from adaptive_control_project.retarget_pipeline.tasks import Task, load_task


def test_default_sequence_conversion_reproduces_every_canonical_field(tmp_path):
    original = load_task('pickup_the_cup')
    dataset = original.manifest['dataset']
    raw = original.path(dataset['sequence_directory'])
    processed = original.path(original.manifest['sequences']['right'])
    if not raw.exists() or not processed.exists():
        pytest.skip('Local DexYCB annotation inputs are absent')
    for relative in (dataset['sequence_directory'], dataset['calibration_directory']):
        shutil.copytree(original.path(relative), tmp_path / relative)
    task = Task(tmp_path, original.manifest)
    converted = convert_task_sequence(task)
    with np.load(converted, allow_pickle=False) as result, np.load(processed, allow_pickle=False) as expected:
        assert set(result.files) == set(expected.files)
        for name in expected.files:
            np.testing.assert_array_equal(result[name], expected[name], err_msg=name)
        assert json.loads(str(result['source']))['sequence'] == '20200709_151032'
        assert result['keypoints'].shape == (62, 21, 3)
    assert original.manifest['hands'] == ['right']
    assert len(list((original.root / 'data/processed').glob('*.npz'))) == 1
