from ament_flake8.main import main_with_errors


def test_flake8():
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, '\n'.join(errors)
