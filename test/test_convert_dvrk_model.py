import importlib.util
from pathlib import Path


_SCRIPT = Path(__file__).parents[1] / "scripts" / "convert_dvrk_model.py"
_SPEC = importlib.util.spec_from_file_location("convert_dvrk_model", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


def test_strip_physics_api_schemas_retains_mesh_collision_api():
    line = 'prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMeshCollisionAPI"]\n'

    assert _MODULE._strip_physics_api_schemas(line) == 'prepend apiSchemas = ["PhysicsMeshCollisionAPI"]\n'


def test_strip_physics_api_schemas_removes_physics_only_entry():
    assert _MODULE._strip_physics_api_schemas('prepend apiSchemas = ["PhysicsRigidBodyAPI"]\n') is None