"""Bounded, same-capture diagnostic image transport, with no live hardware."""
import base64
import copy
import json
from pathlib import Path
import struct
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/rescue_vision'))
from rescue_vision.road_visualization import (RoadVisualizationCache, make_visualization,
    jpeg_dimensions, MAX_MESSAGE_BYTES, MAX_METADATA_BYTES)


def unpack(packet):
    count=struct.unpack('!I',packet[:4])[0]
    return json.loads(packet[4:4+count]),packet[4+count:]


class Clock:
    value=1000.
    def __call__(self): return self.value


class RoadVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock()
        self.cache=RoadVisualizationCache(monotonic=self.clock, wall=self.clock)
        self.pixels=np.full((720,1280,3),30,np.uint8)
        cv2.rectangle(self.pixels,(100,200),(400,500),(240,240,240),-1)
        identity=dict(capture_ros_s=1000.,sequence='1000000000000',
                      frame_id='physical-left:1000000000000',source_topic='/image_right_raw')
        self.results=dict(road=dict(identity,image_size=[480,270],detected=True,bbox=[20,30,200,220]),
                          traffic=dict(identity,image_size=[640,360],status='unknown'))
        self.value=make_visualization(self.pixels,capture_ros_s=1000.,sequence='1000000000000',
            source_topic='/image_right_raw',results=self.results)

    def update(self, value=None):
        return self.cache.update(json.dumps(self.value if value is None else value))

    def test_producer_preserves_pixels_and_result_coordinates(self):
        before=self.pixels.copy()
        self.value=make_visualization(self.pixels,capture_ros_s=1000.,sequence='1000000000000',
            source_topic='/image_right_raw',results=self.results)
        np.testing.assert_array_equal(before,self.pixels)
        self.assertEqual(self.value['frame']['road'],self.results['road'])
        self.results['road']['bbox'][0]=999
        self.assertEqual(self.value['frame']['road']['bbox'][0],20)
        jpeg=base64.b64decode(self.value['jpeg_base64'])
        self.assertEqual(jpeg_dimensions(jpeg),(640,360))
        self.assertEqual(cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR).shape,(360,640,3))

    def test_get_lease_is_required_and_packet_retains_same_frame(self):
        self.assertFalse(self.update())
        meta,jpeg=unpack(self.cache.packet())
        self.assertEqual(meta['state'],'waiting')
        self.assertIsNone(meta['frame']); self.assertEqual(jpeg,b'')
        self.assertTrue(self.cache.wanted())
        self.assertTrue(self.update())
        meta,jpeg=unpack(self.cache.packet())
        self.assertEqual(meta['state'],'live')
        self.assertEqual(meta['frame'],self.value['frame'])
        self.assertEqual(jpeg,base64.b64decode(self.value['jpeg_base64']))
        self.assertFalse(meta['synthetic'])
        self.assertTrue(meta['observation_only'])

    def test_stale_disabled_and_expired_leases_never_return_old_pixels(self):
        self.cache.packet(); self.assertTrue(self.update())
        self.clock.value+=1.6
        meta,jpeg=unpack(self.cache.packet())
        self.assertEqual(meta['state'],'stale')
        self.assertIsNone(meta['frame']); self.assertEqual(jpeg,b'')
        self.clock.value+=3.1
        self.assertFalse(self.cache.wanted())
        self.assertIsNone(self.cache.latest)
        self.assertEqual(unpack(self.cache.packet())[0]['state'],'waiting')
        meta,jpeg=unpack(self.cache.packet(enabled=False))
        self.assertEqual(meta['state'],'paused')
        self.assertFalse(self.cache.wanted())
        self.assertFalse(meta['stream']['enabled']); self.assertEqual(jpeg,b'')

    def test_synthetic_wrong_capture_and_malformed_packets_are_rejected(self):
        self.cache.packet()
        changes=[lambda v:v.update(synthetic=True),
                 lambda v:v['frame'].update(synthetic=True),
                 lambda v:v['frame'].update(width=639),
                 lambda v:v['frame'].update(coordinate_frame='rectified'),
                 lambda v:v['frame'].update(capture_ros_s=float('nan')),
                 lambda v:v['frame']['road'].update(capture_ros_s=999.),
                 lambda v:v['frame']['traffic'].update(sequence='other'),
                 lambda v:v['frame']['road'].update(frame_id='other'),
                 lambda v:v['frame']['traffic'].update(image_size=[False,360]),
                 lambda v:v['frame']['road'].update(source_topic='/wrong_eye'),
                 lambda v:v.update(jpeg_base64='not-base64!'),
                 lambda v:v.update(jpeg_base64=base64.b64encode(b'\xff\xd8bad\xff\xd9').decode()),
                 lambda v:v['frame'].update(extra='X'*MAX_METADATA_BYTES)]
        for change in changes:
            with self.subTest(change=changes.index(change)):
                value=copy.deepcopy(self.value); change(value)
                self.assertFalse(self.update(value))
        for data in ('bad json','[]','X'*(MAX_MESSAGE_BYTES+1)):
            self.assertFalse(self.cache.update(data))
        self.assertIsNone(self.cache.latest)

    def test_stale_future_duplicate_packets_cannot_refresh_cache(self):
        self.cache.packet()
        self.clock.value=999.9
        self.assertFalse(self.update())
        self.clock.value=1000.
        self.assertTrue(self.update())
        self.clock.value=1001.
        self.assertFalse(self.update())
        self.assertEqual(self.cache.latest[2],1000.)
        self.clock.value=1001.6
        self.assertFalse(self.update())
        self.assertEqual(unpack(self.cache.packet())[0]['state'],'stale')


if __name__=='__main__': unittest.main()
