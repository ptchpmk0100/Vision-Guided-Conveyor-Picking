"""World-frame dimensions for the additive Panda parcel cell (metres)."""
from dataclasses import dataclass
import math
import os

FLOOR_Z = -0.350
DECK_Z = 0.295
PARCEL_SIZE = (0.080, 0.050, 0.050)
PARCEL_Z = DECK_Z + PARCEL_SIZE[2] / 2
PICK_X = 0.560
STAGING_Y = (0.110, 0.0, -0.110)
QUEUE_Y = (-0.300, -0.700, -1.100)
CONTINUOUS_QUEUE_Y = (-0.300, -1.500, -2.700)
CONTINUOUS_SPEED = .060
# The original conveyor deck runs from y=-3.6 to y=+1.4 (5.0 m) and is never
# lengthened. Queued boxes start at y=-0.3 and the last one stays 0.2 m inside
# the upstream end, so a bigger batch means a tighter pitch, not a longer belt.
BELT_DECK_Y = (-3.6, 1.4)
QUEUE_START_Y = -0.300
QUEUE_END_MARGIN = 0.200
QUEUE_LAST_Y_LIMIT = BELT_DECK_Y[0] + QUEUE_END_MARGIN
# Gripper clearance along the belt (fingers close along world y). Finger mesh
# reaches 26.4 mm beyond its joint, so fully open (0.04) its outer edge is
# 66.4 mm from the box centre. Axis-aligned box is 50 mm; allow +-15 deg yaw.
FINGER_OUTER_REACH = 0.0664
GRIP_CLEARANCE = 0.035
MIN_PITCH = 0.14
EXTENDED_COUNTS = (7, 8, 10)
TOTE_XY = (0.320, -0.320)
TOTE_SIZE = (0.240, 0.240)
TOTE_WALL = 0.008
TOTE_RIM_Z = 0.375
SLOTS = ((0.260, -0.360), (0.380, -0.360), (0.320, -0.270))
BELT_TOPIC = '/parcel_cell/belt_speed'
DETECTION_TOPIC = '/parcel_cell/detections'
MOVING_DETECTION_TOPIC = '/parcel_cell/moving_detections'
PARCEL_NAMES = tuple(f'parcel_{i}' for i in range(1, 4))


@dataclass(frozen=True)
class ParcelBatch:
    count: int
    pitch: float
    slots: tuple
    tote_xy: tuple
    tote_size: tuple
    camera_hfov: float

    @property
    def extended(self):
        return self.count != 3

    @property
    def names(self):
        return tuple(f'parcel_{i}' for i in range(1,self.count+1))

    @property
    def queue_y(self):
        return tuple(QUEUE_START_Y-i*self.pitch for i in range(self.count))

    @property
    def tote_region(self):
        x,y=self.tote_xy;w,h=self.tote_size
        return (x-w/2+TOTE_WALL,x+w/2-TOTE_WALL,
                y-h/2+TOTE_WALL,y+h/2-TOTE_WALL)

    def contains(self,x,y,yaw=0.,margin=.002):
        hx=(abs(math.cos(yaw))*PARCEL_SIZE[0]+abs(math.sin(yaw))*PARCEL_SIZE[1])/2
        hy=(abs(math.sin(yaw))*PARCEL_SIZE[0]+abs(math.cos(yaw))*PARCEL_SIZE[1])/2
        a,b,c,d=self.tote_region
        return a+hx+margin<=x<=b-hx-margin and c+hy+margin<=y<=d-hy-margin


def fit_pitch(count):
    """Largest 1 mm pitch that keeps `count` boxes on the original belt."""
    if count < 2:
        raise ValueError('A batch needs at least two parcels')
    return math.floor((QUEUE_START_Y-QUEUE_LAST_Y_LIMIT)/(count-1)*1000)/1000


YAW_ALLOWANCE_HALF_EXTENT = 0.0345   # half of 50 mm x cos15 + 80 mm x sin15


def clearance_at(pitch):
    """Free space (m) between an open finger's outer edge and the next box's near face."""
    return pitch-YAW_ALLOWANCE_HALF_EXTENT-FINGER_OUTER_REACH


def batch_layout(count=3,pitch=None):
    """Three-box incumbent, or a 7/8/10-box single-layer profile on the SAME belt."""
    if count not in (3,)+EXTENDED_COUNTS:
        raise ValueError('Supported parcel counts are 3, 7, 8 and 10')
    if pitch is None:pitch=1.2 if count==3 else fit_pitch(count)
    if not math.isfinite(pitch):
        raise ValueError('Parcel pitch must be finite')
    if count==3:
        if not .6<=pitch<=1.2:
            raise ValueError('Parcel pitch must be finite and in [.6,1.2] metres')
        return ParcelBatch(count,pitch,SLOTS,TOTE_XY,TOTE_SIZE,1.047)
    if pitch<MIN_PITCH-1e-9:
        raise ValueError(f'Parcel pitch {pitch} m leaves less than {GRIP_CLEARANCE*1000:.0f} mm '
                         f'gripper clearance; minimum is {MIN_PITCH} m')
    if QUEUE_START_Y-(count-1)*pitch < QUEUE_LAST_Y_LIMIT-1e-9:
        raise ValueError(f'{count} parcels at pitch {pitch} m do not fit on the original belt '
                         f'(last box must stay at y>={QUEUE_LAST_Y_LIMIT})')
    # Work from the belt side toward the left, preserving a 20 mm x gap.
    slots=tuple((x,y) for x in (.37,.27,.17,.07,-.03) for y in (-.37,-.27))[:count]
    return ParcelBatch(count,pitch,slots,(.16,-.32),(.54,.24),math.radians(76))


def active_batch():
    """Run metadata inherited by sensor/independent observer subprocesses."""
    value=os.environ.get('TWIN_PARCEL_PITCH')
    return batch_layout(int(os.environ.get('TWIN_PARCEL_COUNT','3')),
                        None if value is None else float(value))


def footprints_clear(first,second,margin=.002):
    """Separating-axis check of two actual rotated carton footprints."""
    x1,y1,a=first;x2,y2,b=second
    axes=((math.cos(a),math.sin(a)),(-math.sin(a),math.cos(a)),
          (math.cos(b),math.sin(b)),(-math.sin(b),math.cos(b)))
    for ax,ay in axes:
        def radius(angle):
            return (PARCEL_SIZE[0]*abs(ax*math.cos(angle)+ay*math.sin(angle))+
                    PARCEL_SIZE[1]*abs(-ax*math.sin(angle)+ay*math.cos(angle)))/2
        if abs((x2-x1)*ax+(y2-y1)*ay)>=radius(a)+radius(b)+margin:
            return True
    return False


def in_pick_region(x, y):
    return 0.505 <= x <= 0.615 and -0.180 <= y <= 0.180


def inside_tote(x, y, yaw=0.0, margin=0.002):
    """Full rotated parcel footprint must clear the inner tote walls."""
    import math
    hx = (abs(math.cos(yaw)) * PARCEL_SIZE[0]
          + abs(math.sin(yaw)) * PARCEL_SIZE[1]) / 2
    hy = (abs(math.sin(yaw)) * PARCEL_SIZE[0]
          + abs(math.cos(yaw)) * PARCEL_SIZE[1]) / 2
    return (abs(x - TOTE_XY[0]) + hx + margin <= TOTE_SIZE[0] / 2 - TOTE_WALL
            and abs(y - TOTE_XY[1]) + hy + margin <= TOTE_SIZE[1] / 2 - TOTE_WALL)
