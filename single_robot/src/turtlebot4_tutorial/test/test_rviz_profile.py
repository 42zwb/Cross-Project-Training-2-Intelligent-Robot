from pathlib import Path

import yaml


def test_navigation_uses_standard_goal_topic_without_unsafe_nav2_panel():
    path = Path(__file__).resolve().parents[1] / 'config/navigation.rviz'
    profile = yaml.safe_load(path.read_text())
    assert all(not p['Class'].startswith('nav2_rviz_plugins/') for p in profile['Panels'])
    tools = profile['Visualization Manager']['Tools']
    goal = next(t for t in tools if t['Class'] == 'rviz_default_plugins/SetGoal')
    assert goal['Topic']['Value'] == 'goal_pose'
