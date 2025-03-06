import yaml
from pathlib import Path

def load_yaml(file_name):
    """Load YAML from the utils folder."""
    file_path = Path(__file__).parent / file_name
    with open(file_path, "r") as file:
        return yaml.safe_load(file)