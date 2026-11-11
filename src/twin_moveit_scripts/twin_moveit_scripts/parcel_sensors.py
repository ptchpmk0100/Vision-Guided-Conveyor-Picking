"""Add sensors to the temporary parcel URDF without changing physical geometry."""
import xml.etree.ElementTree as ET


def add_pad_sensors(text):
    root = ET.fromstring(text)
    for side, link in (('left', 'panda_leftfinger'), ('right', 'panda_rightfinger')):
        collision = root.find(f"link[@name='{link}']/collision")
        if collision is None:
            raise RuntimeError('Missing physical finger collision: '+link)
        name = link
        collision.set('name', name)
        gazebo = ET.SubElement(root, 'gazebo', reference=link)
        sensor = ET.SubElement(gazebo, 'sensor', name='parcel_'+side+'_contact', type='contact')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '100'
        ET.SubElement(sensor, 'topic').text = '/parcel_cell/contact/'+side
        contact = ET.SubElement(sensor, 'contact')
        ET.SubElement(contact, 'collision').text = name+'_collision'
    return ET.tostring(root, encoding='unicode')
