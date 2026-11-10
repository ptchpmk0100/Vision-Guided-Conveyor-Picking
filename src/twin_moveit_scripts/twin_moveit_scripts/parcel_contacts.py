"""Read real Gazebo pad sensors; never query simulator object poses for control."""
import threading
import time

from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node
from twin_moveit_scripts.parcel_grasp import parcel_from_collision

CONTACT_TOPICS = {side: f'/world/empty/model/panda/link/panda_{side}finger/sensor/parcel_{side}_contact/contact'
                  for side in ('left','right')}


def contact_packet(message, side):
    parcels = set()
    foreign = False
    names = []
    for contact in message.contact:
        a, b = contact.collision1.name, contact.collision2.name
        names.append((a, b))
        if 'panda_'+side+'finger' in a:
            other = b
        elif 'panda_'+side+'finger' in b:
            other = a
        else:
            foreign = True
            continue
        parcel = parcel_from_collision(other)
        if parcel is None:
            foreign = True
        else:
            parcels.add(parcel)
    if foreign:
        parcels.add('foreign_contact')
    return {'stamp_ns': message.header.stamp.sec*1000000000+message.header.stamp.nsec,
            'received_wall': time.monotonic(), 'parcels': frozenset(parcels), 'names': names}


class PadContacts:
    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()
        self.packets = {}
        for side in ('left', 'right'):
            if not self.node.subscribe(Contacts, CONTACT_TOPICS[side],
                                       lambda msg, s=side: self.callback(s, msg)):
                raise RuntimeError('Cannot subscribe to '+side+' pad contact sensor')

    def callback(self, side, message):
        packet = contact_packet(message, side)
        with self.lock:
            self.packets[side] = packet

    def sample(self, now_ns):
        with self.lock:
            packets = dict(self.packets)
        left, right = packets.get('left'), packets.get('right')
        if left is None or right is None:
            return frozenset(), frozenset(), float('inf')
        age = max((now_ns-p['stamp_ns'])/1e9 for p in (left, right))
        if any(time.monotonic()-p['received_wall'] > .5 for p in (left, right)):
            age = float('inf')
        return left['parcels'], right['parcels'], age

    def close(self):
        for side in ('left', 'right'):
            self.node.unsubscribe(CONTACT_TOPICS[side])
