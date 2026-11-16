"""Selection policy explicitly updated by the user: completion and clearance first."""
def apply_completion_policy(r):
    r['accuracy_within_previous_threshold']=r.get('flight_lio_max_m',999)<=.5 and r.get('flight_lio_rms_m',999)<=.25
    r['strict_eligible']=bool(r.get('accepted')and r.get('gui_verified')and r.get('uninterrupted_mission')and r.get('speed_matches_setting'))
    physical=bool(r.get('completed')and r.get('landed')and not r.get('faults')and r.get('gui_verified')and r.get('uninterrupted_mission')and r.get('rate_only_during_tracking')and max(r.get('physical_goal_min_distance_m',[999]))<=.8 and r.get('conservative_body_clearance_m',-999)>=.1 and r.get('floor_clearance_bound_m',-999)>=.1 and r.get('odom_max_sim_gap_s',999)<=.05 and (r.get('odom_max_wall_gap_s')or 999)<=.2)
    r['selection_policy']='user_completion_clearance_priority_20261113'
    r['healthy_complete']=physical;r['eligible']=physical and bool(r.get('speed_matches_setting'))
    r['ranking_time_sim_s']=r.get('tracking_sim_s')if r['eligible']else None
    return r
