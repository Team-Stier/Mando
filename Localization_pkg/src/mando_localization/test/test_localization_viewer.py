#!/usr/bin/env python3
"""Display invariants shared by live subscriptions and recorded results."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
import rospy
import rosbag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

spec = importlib.util.spec_from_file_location('viewer', Path(__file__).resolve().parents[1]/'scripts/localization_viewer.py')
v = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v)

class ViewerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root/'yongin_route_project.json').write_text(json.dumps({'origin':{'lat':37.,'lng':127.}}))
        (self.root/'route.csv').write_text('route_id,route_name,closed,index,latitude,longitude,east_m,north_m,distance_m,path_yaw_rad\n1,test,0,0,37,127,0,0,0,0\n1,test,0,1,37,127,10,10,14,0\n')
        self.local = self.odom(4.,6.,math.radians(60))
        self.glob = self.odom(7.,8.,math.radians(-20))
        self.gps = NavSatFix(); self.gps.status.status=0
        self.gps.latitude=37.;self.gps.longitude=127.

    def tearDown(self): self.tmp.cleanup()

    @staticmethod
    def odom(x,y,yaw):
        msg=Odometry();msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        return msg

    def test_common_translation_preserves_yaw_and_separation(self):
        model=v.SceneData(self.root)
        for key,msg in [('local',self.local),('global',self.glob),('gps',self.gps)]:
            model.ingest(key,msg,100.,True)
        np.testing.assert_allclose(model.points('local')[0],[0,0,math.radians(60)])
        np.testing.assert_allclose(model.points('global')[0],[3,2,math.radians(-20)])
        self.assertEqual(self.local.pose.pose.position.x,4.)

    def test_live_recorded_same_input_same_result(self):
        live=v.SceneData(self.root)
        bagpath=self.root/'computed.bag'
        with rosbag.Bag(str(bagpath),'w') as bag:
            for i,(key,msg) in enumerate([('local',self.local),('gps',self.gps),('global',self.glob)]):
                stamp=100+i*.1
                live.ingest(key,msg,stamp,True)
                bag.write(v.TOPICS[key],msg,rospy.Time.from_sec(stamp))
        recorded=v.load_data(bagpath,None,self.root,200000)
        for key in ('local','global','gps'):
            np.testing.assert_allclose(live.points(key),recorded.points(key))
            np.testing.assert_allclose(live.times[key],recorded.times[key])

    def test_clock_rewind_clears_old_anchor(self):
        model=v.SceneData(self.root)
        model.ingest('local',self.local,100,True);model.ingest('gps',self.gps,101,True)
        model.ingest('local',self.glob,90,True)
        self.assertIsNone(model.shift);self.assertEqual(model.times['local'],[0.])
        self.assertFalse(model.data['gps']);self.assertEqual(model.summary['clock_resets'],1)

    def test_invalid_gps_cannot_anchor(self):
        model=v.SceneData(self.root);model.ingest('local',self.local,100,True)
        self.gps.latitude=91;model.ingest('gps',self.gps,100,True)
        self.assertIsNone(model.shift)

    def test_bounded_buffer_and_earlier_selection(self):
        model=v.SceneData(self.root,limit=3)
        for i in range(6): model.ingest('local',self.local,100+i,True)
        self.assertEqual(len(model.data['local']),3)
        self.assertEqual(model.summary['buffer_dropped'],3)
        self.assertEqual(v.bisect.bisect_right(model.times['local'],4.),2)

if __name__ == '__main__': unittest.main()
