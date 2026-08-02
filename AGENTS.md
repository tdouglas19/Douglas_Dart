# Engineering contribution rules

- Use SI units internally and include the unit in public names.
- Keep user requirements distinct from provisional engineering assumptions.
- Do not add an unexplained constant to make a result look plausible.
- Every new physics model needs at least one limiting-case or conservation test.
- Preserve instantaneous propulsion output; perform cycle averaging explicitly.
- Treat OpenVSP/VSPAERO as external-aerodynamics tooling only.
- Do not commit generated `.vsp3`, mesh, polar, plot, or CSV files unless a review
  specifically calls for a frozen validation artifact.
- A green unit test proves implementation consistency, not physical validation.
