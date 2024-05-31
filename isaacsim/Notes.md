To run RoveTask with a random policy:
- Update asset path in `robots/articulation/rove.py`
- With isaacsim python run `python isaacsim/iniedo/scripts/random_policy.py task=Rove`

Current issue:
- Segmentation fault while initial world.reset cause by rididbody joint views from `husky_view`