
Run:
py server.py

Open:
http://localhost:8000/

Python file layout:
- server.py         -> entry point only
- web_server.py     -> HTTP routes / frontend serving
- app_state.py      -> runtime state + control logic
- sensor_backend.py -> hardware / simulation wrapper around test.py
- models.py         -> shared dataclasses
- test.py           -> unchanged team-provided sensor code
