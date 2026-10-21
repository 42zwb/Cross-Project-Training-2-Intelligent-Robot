from pathlib import Path

import yaml


def test_nav2_keeps_navfn_default_and_exposes_smac_astar():
    config_path = (Path(__file__).parent.parent / 'config' / 'nav2_params.yaml')
    config = yaml.safe_load(config_path.read_text())
    planner = config['planner_server']['ros__parameters']

    assert planner['planner_plugins'][0] == 'GridBased'
    assert planner['GridBased']['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert 'Smac2D' in planner['planner_plugins']
    assert planner['Smac2D']['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert planner['Smac2D']['allow_unknown'] is True
