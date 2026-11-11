#!/usr/bin/env python3
"""Generate the parcel SDF assets from explicit primitive geometry; no network assets."""
import math
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/twin_moveit_scripts'))
from twin_moveit_scripts.parcel_layout import (  # noqa: E402
    BELT_TOPIC, DECK_Z, FLOOR_Z, PARCEL_SIZE, TOTE_XY,
)

SILVER = '0.54 0.58 0.61 1'
DARK = '0.16 0.19 0.21 1'
BLACK = '0.055 0.062 0.070 1'
BLUE = '0.37 0.68 0.73 1'
YELLOW = '0.90 0.74 0.08 1'


def add(parent, tag, value=None, **attrs):
    el = ET.SubElement(parent, tag, attrs)
    if value is not None:
        el.text = str(value)
    return el


def vector(values):
    return ' '.join(f'{v:.7g}' for v in values)


def model(world, name, pose=(0, 0, 0, 0, 0, 0), static=True):
    m = add(world, 'model', name=name)
    add(m, 'static', str(static).lower())
    add(m, 'pose', vector(pose))
    return m, add(m, 'link', name='body')


def solid(link, name, xyz, size, colour=SILVER, collide=True, rpy=(0, 0, 0),
          shape='box'):
    for tag in (('collision', 'visual') if collide else ('visual',)):
        el = add(link, tag, name=name + ('_visual' if tag == 'visual' else ''))
        add(el, 'pose', vector((*xyz, *rpy)))
        geom = add(add(el, 'geometry'), shape)
        if shape == 'box':
            add(geom, 'size', vector(size))
        else:
            add(geom, 'radius', size[0])
            add(geom, 'length', size[1])
        if tag == 'visual':
            mat = add(el, 'material')
            add(mat, 'ambient', colour)
            add(mat, 'diffuse', colour)
            add(mat, 'specular', '0.15 0.15 0.15 1')
        else:
            ode = add(add(add(el, 'surface'), 'friction'), 'ode')
            add(ode, 'mu', 1.0)
            add(ode, 'mu2', 1.0)


def write(path, tree):
    ET.indent(tree, space='  ')
    path.write_text('<?xml version="1.0"?>\n' + ET.tostring(tree, encoding='unicode') + '\n')


def camera(world, name, pose, topic, width=1280, height=720):
    _, link = model(world, name, pose)
    sensor = add(link, 'sensor', name=name, type='camera')
    add(sensor, 'always_on', 'true')
    add(sensor, 'update_rate', 2)
    add(sensor, 'topic', topic)
    cam = add(sensor, 'camera')
    add(cam, 'horizontal_fov', 1.0472)
    im = add(cam, 'image')
    add(im, 'width', width)
    add(im, 'height', height)
    add(im, 'format', 'R8G8B8')
    clip = add(cam, 'clip')
    add(clip, 'near', 0.05)
    add(clip, 'far', 20)


def generate():
    root = ET.Element('sdf', version='1.9')
    w = add(root, 'world', name='empty')
    physics = add(w, 'physics', name='1ms', type='ignored')
    add(physics, 'max_step_size', 0.001)
    add(physics, 'real_time_factor', 1)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        p = add(w, 'plugin', filename=f'gz-sim-{filename}-system',
                name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            add(p, 'render_engine', 'ogre2')
    scene = add(w, 'scene')
    add(scene, 'grid', 'false')
    add(scene, 'ambient', '0.35 0.35 0.35 1')
    add(scene, 'background', '0.055 0.065 0.080 1')
    light = add(w, 'light', type='directional', name='sun')
    add(light, 'pose', '0 0 10 0 0 0')
    add(light, 'cast_shadows', 'true')
    add(light, 'diffuse', '0.8 0.8 0.8 1')
    add(light, 'specular', '0.2 0.2 0.2 1')
    add(light, 'direction', '-0.5 0.1 -0.9')
    att = add(light, 'attenuation')
    for key, val in [('range', 1000), ('constant', .9), ('linear', .01), ('quadratic', .001)]:
        add(att, key, val)
    _, ground = model(w, 'ground_plane')
    solid(ground, 'floor', (0, 0, FLOOR_Z - .025), (20, 20, .05), '0.24 0.26 0.28 1')
    _, backdrop = model(w, 'rear_building_wall')
    solid(backdrop, 'wall', (-1.65, 0, 1.10), (.06, 7, 2.9), '0.045 0.052 0.065 1')
    solid(backdrop, 'return', (-.5, 2.15, 1.10), (2.4, .06, 2.9), '0.045 0.052 0.065 1')

    # Local +X belt travel becomes world +Y. The installed Harmonic conveyor
    # example uses TrackController's surface contact motion on a static link.
    belt, link = model(w, 'pick_belt', (.60, 0, 0, 0, 0, math.pi / 2))
    solid(link, 'deck', (0, 0, DECK_Z - .015), (2.8, .24, .03), BLACK)
    p = add(belt, 'plugin', filename='gz-sim-track-controller-system',
            name='gz::sim::systems::TrackController')
    add(p, 'link', 'body')
    add(p, 'velocity_topic', BELT_TOPIC)
    add(p, 'odometry_publish_frequency', 5)
    # Fixed rails/legs live on a separate static body so their contacts cannot convey.
    _, frame = model(w, 'conveyor_frame', (.60, 0, 0, 0, 0, math.pi / 2))
    for y in (-.143, .143):
        solid(frame, f'rail_{y}', (0, y, .280), (2.9, .028, .07), SILVER)
        solid(frame, f'lower_{y}', (0, y, -.15), (2.75, .025, .045), DARK)
        for x in (-1.20, -.40, .40, 1.20):
            solid(frame, f'leg_{x}_{y}', (x, y, -.025), (.045, .045, .65), SILVER)
            solid(frame, f'foot_{x}_{y}', (x, y, FLOOR_Z + .008), (.085, .08, .016), DARK)
    for x in (-1.4, 1.4):
        solid(frame, f'roller_{x}', (x, 0, .264), (.032, .24), SILVER,
              rpy=(math.pi / 2, 0, 0), shape='cylinder')

    _, pedestal = model(w, 'robot_pedestal')
    solid(pedestal, 'foot', (0, 0, -.326), (.62, .62, .048), DARK)
    solid(pedestal, 'body', (0, 0, -.180), (.50, .50, .244), SILVER)
    solid(pedestal, 'top', (0, 0, -.029), (.56, .56, .058), '0.62 0.65 0.67 1')
    for y in (-.18, .18):
        solid(pedestal, f'recess_{y}', (.251, y, -.18), (.002, .07, .10), DARK, False)

    tx, ty = TOTE_XY
    _, station = model(w, 'tote_station')
    solid(station, 'top', (tx, ty, .273), (.27, .27, .024), SILVER)
    for x in (tx - .105, tx + .105):
        for y in (ty - .105, ty + .105):
            # Offset the inner floor leg past the plinth's x=.31 edge.
            # Its top still lies under the station, with the load inside the
            # four-foot support polygon; no leg passes through the pedestal.
            leg_x = .350 if x < tx and y > ty else x
            solid(station, f'leg_{x}_{y}', (leg_x, y, -.0445), (.023, .023, .611), SILVER)
    _, tote = model(w, 'collection_tote_blue')
    solid(tote, 'floor', (tx, ty, .290), (.24, .24, .01), BLUE)
    for side in (-1, 1):
        solid(tote, f'wall_x_{side}', (tx + side * .116, ty, .335), (.008, .24, .08), BLUE)
        solid(tote, f'wall_y_{side}', (tx, ty + side * .116, .335), (.24, .008, .08), BLUE)
        # Rim remains outside the interior footprint.
        solid(tote, f'rim_x_{side}', (tx + side * .120, ty, .371), (.014, .254, .008), BLUE)
        solid(tote, f'rim_y_{side}', (tx, ty + side * .120, .371), (.254, .014, .008), BLUE)

    _, cabinet = model(w, 'cabinet')
    solid(cabinet, 'body', (-.70, .10, .575), (.38, .64, 1.85), SILVER)
    solid(cabinet, 'door', (-.504, .10, .575), (.012, .58, 1.75), '0.64 0.67 0.69 1')
    solid(cabinet, 'vent_panel', (-.496, .10, .88), (.008, .43, .40), DARK, False)
    for i in range(14):
        solid(cabinet, f'vent_{i}', (-.49, .10, .703 + i * .027), (.009, .41, .008), SILVER, False)
    solid(cabinet, 'seam', (-.488, .22, .35), (.006, .005, 1.10), DARK, False)
    solid(cabinet, 'handle', (-.474, .25, .52), (.024, .012, .13), DARK, False)

    for name, centre, yaw, length in [('fence_back', (-1.08, 0), 0, 3.25),
                                      ('fence_right', (.03, 1.62), math.pi / 2, 2.22)]:
        _, fence = model(w, name, (*centre, 0, 0, 0, yaw))
        for z in (FLOOR_Z + .10, FLOOR_Z + 1.98):
            solid(fence, f'rail_{z}', (0, 0, z), (.035, length, .04), SILVER)
        for i in range(int(length / .09) + 1):
            y = -length / 2 + i * .09
            solid(fence, f'bar_{i}', (0, y, FLOOR_Z + 1.04), (.012, .012, 1.86), SILVER)
        for y in (-length / 2, 0, length / 2):
            solid(fence, f'post_{y}', (0, y, FLOOR_Z + 1.0), (.055, .055, 2), SILVER)
            solid(fence, f'foot_{y}', (0, y, FLOOR_Z + .01), (.14, .14, .02), DARK)

    _, feeder = model(w, 'feeder_belt')
    solid(feeder, 'deck', (.10, 1.14, .28), (.70, .24, .03), BLACK)
    for y in (1.0, 1.28):
        solid(feeder, f'rail_{y}', (.10, y, .28), (.75, .025, .07), SILVER)
        for x in (-.20, .40):
            solid(feeder, f'leg_{x}_{y}', (x, y, -.025), (.04, .04, .65), SILVER)
    for x in (-.20, .40):
        solid(feeder, f'crossbrace_{x}', (x, 1.14, .25), (.05, .32, .03), SILVER)
    # Feeder ends at x=.45, before the main belt's x=.48 edge: no transfer claim.
    _, arm = model(w, 'camera_arm')
    solid(arm, 'post', (.40, .86, .425), (.045, .045, 1.55), SILVER)
    solid(arm, 'beam', (.40, .43, 1.21), (.035, .90, .035), SILVER)
    solid(arm, 'sensor', (.40, .0, 1.24), (.065, .07, .025), DARK, False)
    for i in range(40):
        angle = i * math.tau / 40
        solid(arm, f'orange_ring_{i}', (.05 + .13 * math.cos(angle),
              .82 + .13 * math.sin(angle), .30), (.024, .023, .055),
              '0.68 0.29 0.06 1', False, rpy=(0, 0, angle))
    solid(arm, 'fixture', (.05, .82, .37), (.07, .07, .12), DARK)
    solid(arm, 'fixture_stand', (.05, .82, -.03875), (.09, .09, .6225), DARK)
    solid(arm, 'fixture_crossbrace', (.05, .82, .2725), (.29, .06, .020), DARK)
    solid(arm, 'device_mount', (.05, .82, .295), (.07, .07, .040), DARK)

    _, marks = model(w, 'floor_markings')
    for y in (-.72, .72):
        solid(marks, f'yellow_{y}', (-.25, y, FLOOR_Z + .001), (.90, .035, .002), YELLOW, False)
    solid(marks, 'yellow_back', (-.68, 0, FLOOR_Z + .001), (.035, 1.45, .002), YELLOW, False)
    for i, y in enumerate((-.65, .65)):
        solid(marks, f'pick_mark_{i}', (.76, y, .296), (.025, .16, .001),
              ('0.8 0.08 0.05 1' if i == 0 else '0.08 0.7 0.16 1'), False)

    # Static background parcels have explicit names and are never scored as handled.
    for i, (x, y, z) in enumerate(((.10, 1.14, .32), (.60, .80, .32),
                                  (.60, 1.12, .32))):
        _, carton = model(w, f'display_carton_{i}')
        solid(carton, 'body', (x, y, z), (.10, .065, .05), '0.65 0.47 0.29 1')
        solid(carton, 'tape', (x, y, z + .0255), (.015, .065, .001), '0.78 0.65 0.45 1', False)

    overview = (2.60, 1.08, 1.88, 0, .50, -2.706)
    camera(w, 'presentation_camera', overview, '/parcel_cell/overview')
    camera(w, 'side_camera', (2.3, -2.1, 1.5, 0, .34, 2.37), '/parcel_cell/side')
    gui = add(w, 'gui', fullscreen='false')
    plugin = add(gui, 'plugin', filename='MinimalScene', name='3D View')
    config = add(plugin, 'gz-gui')
    add(config, 'title', '3D View')
    add(config, 'property', 'false', type='bool', key='showTitleBar')
    add(plugin, 'engine', 'ogre2')
    add(plugin, 'scene', 'scene')
    add(plugin, 'camera_pose', vector(overview))
    add(plugin, 'background_color', '.055 .065 .08')
    for filename in ('GzSceneManager', 'InteractiveViewControl', 'CameraTracking',
                     'EntityContextMenuPlugin'):
        add(gui, 'plugin', filename=filename, name=filename)
    for filename in ('WorldControl', 'WorldStats', 'Screenshot'):
        plugin = add(gui, 'plugin', filename=filename, name=filename)
        config = add(plugin, 'gz-gui')
        add(config, 'property', 'false', type='bool', key='showTitleBar')
        add(config, 'property', 'floating', type='string', key='state')
        add(config, 'property', 'false', type='bool', key='resizable')
        if filename == 'WorldControl':
            add(plugin, 'play_pause', 'true')
            add(plugin, 'step', 'true')
            add(plugin, 'start_paused', 'false')
            add(plugin, 'service', '/world/empty/control')
        if filename == 'WorldStats':
            for field in ('sim_time', 'real_time', 'real_time_factor', 'iterations'):
                add(plugin, field, 'true')
    write(ROOT / 'src/twin_action_demo/worlds/industrial_parcel_cell.sdf', root)

    # Continuous feed needs enough physical queue length for one robot cycle per
    # 1.2 m pitch. Extend upstream only; the calibrated pick/tote geometry stays put.
    continuous = copy.deepcopy(root)
    add(continuous.find('world'), 'plugin', filename='gz-sim-contact-system',
        name='gz::sim::systems::Contact')
    for name in ('pick_belt', 'conveyor_frame'):
        body = continuous.find(f"world/model[@name='{name}']/link")
        for item in body:
            if item.tag not in ('collision', 'visual'):
                continue
            label = item.get('name')
            values = list(map(float, item.findtext('pose').split()))
            size = item.find('geometry/box/size')
            if label.startswith(('deck', 'rail_', 'lower_')):
                values[0] -= 1.1
                dims = list(map(float, size.text.split()))
                dims[0] += 2.2
                size.text = vector(dims)
            elif label.startswith('roller_-1.4'):
                values[0] -= 2.2
            item.find('pose').text = vector(values)
        if name == 'conveyor_frame':
            for y in (-.143, .143):
                for x in (-2.0, -2.8, -3.4):
                    solid(body, f'extension_leg_{x}_{y}', (x, y, -.025), (.045, .045, .65), SILVER)
                    solid(body, f'extension_foot_{x}_{y}', (x, y, FLOOR_Z+.008), (.085, .08, .016), DARK)
    write(ROOT / 'src/twin_action_demo/worlds/industrial_parcel_cell_continuous.sdf', continuous)

    asset = ET.Element('sdf', version='1.9')
    _, carton = model(asset, 'parcel', static=False)
    inertia = add(carton, 'inertial')
    mass = .05
    add(inertia, 'mass', mass)
    tensor = add(inertia, 'inertia')
    a, b, c = PARCEL_SIZE
    for name, value in [('ixx', mass*(b*b+c*c)/12), ('iyy', mass*(a*a+c*c)/12),
                        ('izz', mass*(a*a+b*b)/12), ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        add(tensor, name, value)
    solid(carton, 'carton', (0, 0, 0), PARCEL_SIZE, '0.70 0.53 0.35 1')
    solid(carton, 'tape', (0, 0, c/2 + .00015), (.012, b, .0003), '0.79 0.66 0.46 1', False)
    solid(carton, 'label', (.020, .005, c/2 + .0003), (.022, .019, .0003), '0.85 0.84 0.75 1', False)
    for i in range(7):
        solid(carton, f'barcode_{i}', (.012 + i*.0024, .005, c/2 + .0005),
              (.0008, .012, .0001), BLACK, False)
    write(ROOT / 'src/twin_action_demo/models/parcel_box.sdf', asset)


if __name__ == '__main__':
    generate()
