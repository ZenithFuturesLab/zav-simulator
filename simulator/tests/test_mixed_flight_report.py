"""Regression for original waypoint scoring after projected-goal/landing failures."""
import unittest,sys,json,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/super'))
from mixed_flight_report import waypoint_visits,analyze

def row(t,p,state='TRACKING',v=(0.,0.,0.)):
 return dict(stamp=t,p=p,v=v,state=state)

class MissionScoring(unittest.TestCase):
 def test_initial_return_pad_cannot_count_as_return(self):
  v=waypoint_visits([row(1,[0,-50,1.5]),row(2,[0,50,1.5]),row(3,[0,-50,1.5])])
  self.assertEqual([x['stamp']for x in v],[2,3])
 def test_shifted_goal_does_not_relax_original_radius(self):
  v=waypoint_visits([row(1,[.525,50.175,2.325])]);self.assertFalse(any(x['reached']for x in v))
 def test_landing_cannot_complete_waypoint(self):
  v=waypoint_visits([row(1,[0,50,1.5]),row(2,[0,-50,1.5],state='LANDING')]);self.assertTrue(v[0]['reached']);self.assertFalse(v[1]['reached'])
 def test_moving_through_goal_is_not_a_stop(self):
  v=waypoint_visits([row(1,[0,50,1.5],v=(0,1.,0))]);self.assertFalse(v[0]['reached'])
 def test_different_map_cannot_relabel_old_recording(self):
  with tempfile.TemporaryDirectory()as directory:
   p=Path(directory);(p/'metadata.json').write_text(json.dumps({'scenario':'mixed','map_recipe':{'generated_pcd_sha256':'0'*64}}))
   with self.assertRaisesRegex(ValueError,'Current map differs'):analyze(p)

if __name__=='__main__':unittest.main()
