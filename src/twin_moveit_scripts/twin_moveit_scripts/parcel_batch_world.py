"""Generate an isolated 7/8/10-parcel scene (wider tote) on the ORIGINAL conveyor."""
import copy
import xml.etree.ElementTree as ET


def _values(element):return list(map(float,element.text.split()))
def _text(values):return ' '.join(f'{v:.9g}' for v in values)


def _box(link,template,name,center,size):
    for tag in ('collision','visual'):
        item=copy.deepcopy(link.find(f"{tag}[@name='{template}{'_visual' if tag=='visual' else ''}']"))
        if item is None:raise ValueError('Missing box template '+template)
        item.set('name',name+('_visual' if tag=='visual' else ''))
        item.find('pose').text=_text((*center,0,0,0))
        item.find('geometry/box/size').text=_text(size)
        link.append(item)


def batch_world(source,batch):
    root=ET.parse(source).getroot()
    if batch.count==3:return root
    world=root.find('world')
    # The conveyor is NEVER lengthened: the batch is fitted onto the original deck
    # (parcel_layout.batch_layout rejects any pitch whose last box would not fit).
    from twin_moveit_scripts.parcel_layout import QUEUE_LAST_Y_LIMIT
    if batch.queue_y[-1] < QUEUE_LAST_Y_LIMIT-1e-9:
        raise ValueError('Batch does not fit on the original conveyor deck')
    x,y=batch.tote_xy;w,h=batch.tote_size
    tote=world.find("model[@name='collection_tote_blue']/link")
    for item in tote:
        if item.tag not in ('collision','visual'):continue
        name=item.get('name').removesuffix('_visual')
        pose=_values(item.find('pose'));dims=_values(item.find('geometry/box/size'))
        if name=='floor':pose[:2]=[x,y];dims[:2]=[w,h]
        elif name.startswith('wall_x_'):
            side=int(name.split('_')[-1]);pose[:2]=[x+side*(w/2-.004),y];dims[1]=h
        elif name.startswith('wall_y_'):
            side=int(name.split('_')[-1]);pose[:2]=[x,y+side*(h/2-.004)];dims[0]=w
        elif name.startswith('rim_x_'):
            side=int(name.split('_')[-1]);pose[:2]=[x+side*w/2,y];dims[1]=h+.014
        elif name.startswith('rim_y_'):
            side=int(name.split('_')[-1]);pose[:2]=[x,y+side*h/2];dims[0]=w+.014
        else:raise ValueError('Unknown tote primitive '+name)
        item.find('pose').text=_text(pose);item.find('geometry/box/size').text=_text(dims)
    table=world.find("model[@name='tote_station']/link")
    top=table.find("collision[@name='top']");old_leg=next(
        e.get('name') for e in table.findall('collision') if e.get('name').startswith('leg_'))
    _box(table,'top','batch_top',(x,y,.273),(.54,.27,.024))
    # The left/front outrigger clears the robot plinth (|x|<=.31). An elevated
    # beam joins it to the tabletop; four feet surround the tote load centre.
    for i,(lx,ly) in enumerate(((-.07,-.425),(.39,-.425),(-.35,-.215),(.39,-.215))):
        _box(table,old_leg,f'batch_leg_{i}',(lx,ly,-.0445),(.023,.023,.611))
    _box(table,'top','batch_outrigger',(-.02,-.215,.26),(.68,.03,.03))
    for item in list(table):
        if item.tag in ('collision','visual') and not item.get('name').startswith('batch_'):
            table.remove(item)
    return root


def write_batch_world(source,destination,batch):
    root=batch_world(source,batch);ET.indent(root,space='  ')
    ET.ElementTree(root).write(destination,encoding='unicode',xml_declaration=True)
