"""Small atomic per-robot fences that survive autonomy process restarts."""
import json
from pathlib import Path


class Journal:
    def __init__(self, run, robot, component, session):
        self.path = Path(run)/robot/(component+'.json')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.session = session
        self.data = {}
        if self.path.exists():
            value = json.loads(self.path.read_text())
            if isinstance(value, dict) and value.get('session_id') == session:
                self.data = value

    def update(self, **values):
        self.data.update(values, session_id=self.session)
        temp = self.path.with_suffix('.tmp')
        temp.write_text(json.dumps(self.data, allow_nan=False))
        temp.replace(self.path)
