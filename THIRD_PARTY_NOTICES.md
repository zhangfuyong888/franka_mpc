Project-authored source code and documentation are covered by the root MIT
[LICENSE](LICENSE). The bundled third-party model and meshes retain their
Apache-2.0 terms below; the root MIT license does not replace those terms.

# Third-party sources

The bundled Panda MJCF model and meshes originate from Google DeepMind
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), commit
`cda5236f98383f6fbaa4820c6b18dba892cd087f`, directory `franka_emika_panda`.
The upstream model is derived from the public Franka ROS description.
The original Apache-2.0 license is preserved in
[src/panda_mpc/assets/panda/LICENSE](src/panda_mpc/assets/panda/LICENSE).
The upstream explanatory README and original MJCF are retained alongside it.
Original Git blob checksums for all 67 copied meshes are in `provenance.json`.

Changes: seven direct-torque motors replace position servos; inherited actuator
bias/gain defaults removed; finger DOFs/tendon/equality removed with bodies fixed
at 0.03 m per side; mass/inertia/meshes retained; ee_site, home, and scene added.
Modified files carry a change notice. SHA-256 resource manifest is bundled.

mink (Kevin Zakka; Apache-2.0) is a runtime dependency. Its constrained differential
IK API and frame Jacobian conventions informed `ik.py`; its execution layer was
not copied. Reviewed source tag v0.0.13 / commit
`47088abce59efcd54ffe69423c1df3748775f785`.

mjctrl (Kevin Zakka; Apache-2.0) examples diffik_nullspace.py and opspace.py were
reviewed for MuJoCo Jacobian/control patterns, at commit
`f1c82c257248e2e52b0e6c1beaf23c6a2026e4f2`. They are not bundled or copied.

MuJoCo, NumPy, SciPy, OSQP, qpsolvers, PyYAML, Matplotlib and pytest are separate
third-party dependencies, subject to their own licenses. No third-party code or
model is claimed as original work. The optional offline verification script uses
an existing local CasADi/OSQP installation; it is not a production dependency.
