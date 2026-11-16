"""Analytic motion and frame-contract tests for offline flight derivatives."""
import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/super'))
from flight_metrics import kinematics, peaks

META={'imu_position':[.01215,0,.11503],'imu_orientation':[0,22,0]}

def samples(t,v):
    return [{'stamp':float(x),'v':list(y),'state':'TRACKING'}for x,y in zip(t,v)]

class MotionTests(unittest.TestCase):
    def test_vector_derivative_and_peak_time(self):
        t=np.arange(0,4,.01);v=np.column_stack((t*t,3*t,np.zeros(len(t))))
        parts=kinematics(samples(t,v),'lio',META);self.assertEqual(len(parts),1);p=parts[0]
        np.testing.assert_allclose(p['a'],np.column_stack((2*p['t'],np.full(len(p['t']),3),np.zeros(len(p['t'])))),atol=1e-9)
        np.testing.assert_allclose(p['j'],np.tile([2,0,0],(len(p['t']),1)),atol=1e-8)
        self.assertAlmostEqual(peaks(parts,True)['v']['stamp'],p['t'][-1])

    def test_turning_at_constant_speed_has_acceleration_and_jerk(self):
        t=np.arange(0,4,.01);v=np.column_stack((2*np.cos(t),2*np.sin(t),np.zeros(len(t))))
        p=kinematics(samples(t,v),'lio',META)[0]
        np.testing.assert_allclose(np.linalg.norm(p['a'],axis=1),2,atol=.001)
        np.testing.assert_allclose(np.linalg.norm(p['j'],axis=1),2,atol=.003)

    def test_mounted_imu_rotation_and_lever_arm(self):
        t=np.arange(0,4,.01);body=Rotation.from_euler('z',.7*t);mount=Rotation.from_euler('ZYX',META['imu_orientation'],degrees=True);imu=body*mount
        lever=body.apply(META['imu_position']);omega=np.tile([0,0,.7],(len(t),1));vel=np.tile([2,1,0],(len(t),1))
        local_v=imu.inv().apply(vel+np.cross(omega,lever));local_w=imu.inv().apply(omega)
        rows=[{'stamp':float(t[i]),'v':local_v[i].tolist(),'w':local_w[i].tolist(),'q':imu.as_quat()[i].tolist(),'state':'TRACKING'}for i in range(len(t))]
        p=kinematics(rows,'truth',META)[0];np.testing.assert_allclose(p['v'],np.tile([2,1,0],(len(p['t']),1)),atol=1e-12)
        np.testing.assert_allclose(p['a'],0,atol=1e-9);np.testing.assert_allclose(p['j'],0,atol=1e-8)

    def test_gaps_resets_and_invalid_values_do_not_create_spikes(self):
        t=np.r_[np.arange(0,1,.01),np.arange(2,3,.01),np.arange(0,1,.01)]
        v=np.zeros((len(t),3));v[:100]=[1,0,0];v[100:200]=[100,0,0];v[200:]=[1000,0,0];v[240]=np.nan
        parts=kinematics(samples(t,v),'lio',META);self.assertEqual(len(parts),4)
        for p in parts:
            np.testing.assert_allclose(p['a'],0,atol=1e-8);np.testing.assert_allclose(p['j'],0,atol=1e-7)

if __name__=='__main__':unittest.main()
