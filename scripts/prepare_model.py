"""Deterministically derive the torque model from the bundled upstream MJCF."""
from pathlib import Path
import xml.etree.ElementTree as ET
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1] / 'src/panda_mpc/assets/panda'


def derive(root=ROOT):
    tree = ET.parse(root / 'upstream.xml')
    model = tree.getroot()
    # Remove all inherited general-actuator defaults, not just explicit gains.
    for parent in model.iter():
        for node in list(parent):
            if node.tag == 'general' and parent.tag == 'default':
                parent.remove(node)
    for tag in ('actuator', 'tendon', 'equality', 'keyframe'):
        for node in model.findall(tag):
            model.remove(node)
    for name, y in [('left_finger', 0.03), ('right_finger', -0.03)]:
        body = model.find(f'.//body[@name="{name}"]')
        if body is None:
            raise ValueError(f'Missing {name}')
        body.set('pos', f'0 {y} 0.0584')
        for joint in body.findall('joint'):
            body.remove(joint)
    hand = model.find('.//body[@name="hand"]')
    ET.SubElement(hand, 'site', name='ee_site', pos='0 0 0.1034', size='0.008', rgba='0 1 0 1')
    actuators = ET.SubElement(model, 'actuator')
    for i, limit in enumerate([87, 87, 87, 87, 12, 12, 12], 1):
        ET.SubElement(actuators, 'motor', name=f'motor{i}', joint=f'joint{i}',
                      gear='1', ctrllimited='true', ctrlrange=f'{-limit} {limit}',
                      forcelimited='true', forcerange=f'{-limit} {limit}')
    keys = ET.SubElement(model, 'keyframe')
    ET.SubElement(keys, 'key', name='home', qpos='0 -0.6 0 -2.0 0 1.5 0.785398', ctrl='0 0 0 0 0 0 0')
    model.find('option').set('timestep', '0.002')
    ET.indent(tree, space='  ')
    xml = ET.tostring(model, encoding='unicode')
    (root / 'panda.xml').write_text('<!-- Modified for panda-mpc-tracking: direct torque motors, fixed fingers, ee_site, home.\nSource and license: provenance.json and LICENSE. -->\n' + xml + '\n')
    (root / 'scene.xml').write_text('''<!-- Modified project scene: bundled torque Panda and ground plane. See LICENSE/provenance.json. -->
<mujoco model="panda_tracking">
  <include file="panda.xml"/>
  <visual><global azimuth="135" elevation="-25"/></visual>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.24 0.27 0.30 1"/>
  </worldbody>
</mujoco>
''')
    files = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in root.rglob('*') if p.is_file() and p.name != 'model_sha256.json'}
    (root / 'model_sha256.json').write_text(json.dumps(files, indent=2) + '\n')
    print(f'Derived seven-axis model; {len(files)} packaged resources')


if __name__ == '__main__':
    derive()
