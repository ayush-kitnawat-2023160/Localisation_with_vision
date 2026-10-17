#!/usr/bin/env python3
"""Read-only bag inventory and GPS/Lanelet datum consistency report."""
import argparse
import math
import xml.etree.ElementTree as ET
from collections import defaultdict

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

EARTH_A = 6378137.0
EARTH_E2 = 0.0066943799901413165

def utm(lat, lon, zone=43):
    p, l = math.radians(lat), math.radians(lon)
    ep = EARTH_E2/(1-EARTH_E2); l0 = math.radians((zone-1)*6-177)
    n = EARTH_A/math.sqrt(1-EARTH_E2*math.sin(p)**2); t=math.tan(p)**2; c=ep*math.cos(p)**2; a=math.cos(p)*(l-l0)
    m=EARTH_A*((1-EARTH_E2/4-3*EARTH_E2**2/64-5*EARTH_E2**3/256)*p-(3*EARTH_E2/8+3*EARTH_E2**2/32+45*EARTH_E2**3/1024)*math.sin(2*p)+(15*EARTH_E2**2/256+45*EARTH_E2**3/1024)*math.sin(4*p)-(35*EARTH_E2**3/3072)*math.sin(6*p))
    return (.9996*n*(a+(1-t+c)*a**3/6+(5-18*t+t*t+72*c-58*ep)*a**5/120)+500000, .9996*(m+n*math.tan(p)*(a*a/2+(5-t+9*c+4*c*c)*a**4/24+(61-58*t+t*t+600*c-330*ep)*a**6/720)))

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('bag'); parser.add_argument('lanelet_map'); args=parser.parse_args()
    topics=defaultdict(lambda: {'count':0,'first':None,'last':None,'type':''}); gps=[]; types=get_typestore(Stores.ROS1_NOETIC)
    with Reader(args.bag) as bag:
        for connection, stamp, raw in bag.messages():
            entry=topics[connection.topic]; entry['count'] += 1; entry['first']=stamp if entry['first'] is None else entry['first']; entry['last']=stamp; entry['type']=connection.msgtype
            if connection.topic == '/mavros/global_position/global':
                message=types.deserialize_ros1(raw, connection.msgtype); gps.append((message.latitude,message.longitude))
    for topic, entry in sorted(topics.items()):
        duration=max(1,entry['last']-entry['first'])/1e9
        print(f'{topic}\t{entry["type"]}\tcount={entry["count"]}\trate={entry["count"]/duration:.2f}Hz')
    root=ET.parse(args.lanelet_map).getroot(); nodes=[(float(x.attrib['lat']),float(x.attrib['lon'])) for x in root.findall('node')]
    print('lanelet_bbox lat=[%.7f, %.7f] lon=[%.7f, %.7f]' % (min(x[0] for x in nodes),max(x[0] for x in nodes),min(x[1] for x in nodes),max(x[1] for x in nodes)))
    print('gps_bbox     lat=[%.7f, %.7f] lon=[%.7f, %.7f]' % (min(x[0] for x in gps),max(x[0] for x in gps),min(x[1] for x in gps),max(x[1] for x in gps)))
    origin=utm(28.54875901,77.27399459); first=utm(*gps[0]); print('first_gps_in_lanelet_utm_m x=%.2f y=%.2f' % (first[0]-origin[0],first[1]-origin[1]))

if __name__ == '__main__': main()
