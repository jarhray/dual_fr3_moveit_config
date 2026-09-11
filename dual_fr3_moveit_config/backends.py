"""Backend selection shared by MoveIt and task launch entrypoints."""

SIMULATION_BACKENDS = ("gazebo", "maniskill", "fake", "real")
DEFAULT_SIMULATION_BACKEND = "gazebo"


def resolve_simulation_backend(backend: str) -> str:
    if backend not in SIMULATION_BACKENDS:
        raise ValueError(
            f"Unknown simulation_backend: {backend!r}; "
            f"choose one of {', '.join(SIMULATION_BACKENDS)}"
        )
    return backend
