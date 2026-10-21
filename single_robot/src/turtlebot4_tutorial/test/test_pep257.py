from ament_pep257.main import main


def test_pep257():
    assert main(argv=['.']) == 0
