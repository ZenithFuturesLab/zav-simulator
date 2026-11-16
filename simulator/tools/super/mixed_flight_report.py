#!/usr/bin/env python3
"""Offline mixed-map report: original mission criteria, zones and native contacts.

Ground truth and zone labels are used only here, after the flight.
"""
import argparse
import base64
import csv
import hashlib
import html
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

try:
    from .pcd_scene import read_pcd
except ImportError:
    from pcd_scene import read_pcd


def rows(path):
    result, bad = [], 0
    if Path(path).exists():
        for line in Path(path).read_text().splitlines():
            try:
                result.append(json.loads(line))
            except ValueError:
                bad += 1
    return result, bad


def waypoint_visits(lio):
    """A landing or a projected target never substitutes for the original goal."""
    result, after = [], -np.inf
    for goal in ([0., 50., 1.5], [0., -50., 1.5]):
        eligible = [r for r in lio if r['stamp'] > after and r.get('state') == 'TRACKING']
        slow = [r for r in eligible if np.linalg.norm(r['v']) < .7]
        distances = [float(np.linalg.norm(np.array(r['p']) - goal)) for r in slow]
        hit = next((r for r, distance in zip(slow, distances) if distance < .8), None)
        result.append(dict(goal=goal, reached=hit is not None,
                           stamp=hit['stamp'] if hit else None,
                           minimum_distance_with_speed_below_0_7_m=min(distances) if distances else None))
        if hit is None:
            result.append(dict(goal=[0., -50., 1.5], reached=False, stamp=None,
                               note='Return waypoint cannot count before outbound waypoint'))
            return result[:2]
        after = hit['stamp']
    return result


def analyze(directory, contacts=None):
    directory = Path(directory).resolve()
    metadata = json.loads((directory / 'metadata.json').read_text())
    if metadata.get('scenario') != 'mixed':
        raise ValueError('This report requires a mixed scenario recording')
    root = directory.parent if directory.name == 'telemetry' else directory
    ws = Path(__file__).resolve().parents[3]
    assets = ws / 'simulator/assets/super'
    recipe = metadata.get('map_recipe') or json.loads((assets / 'mixed_recipe.json').read_text())
    map_hash = hashlib.sha256((assets / 'mixed_sparse_dense.pcd').read_bytes()).hexdigest()
    if map_hash != recipe['generated_pcd_sha256']:
        raise ValueError('Current map differs from the recording; restore the matching map before reanalysis')
    previous = root / 'mixed-result.json'
    if previous.exists() and json.loads(previous.read_text())['map_sha256'] != map_hash:
        raise ValueError('Map changed since this recording was analyzed; refusing to replace its report')
    samples, malformed = rows(directory / 'samples.jsonl')
    tr = sorted({r['stamp']: r for r in samples if r.get('source') == 'truth'}.values(),
                key=lambda r: r['stamp'])
    li = [r for r in samples if r.get('source') == 'lio' and r.get('state') == 'TRACKING']
    visits = waypoint_visits(li)
    summary = json.loads((directory / 'summary.json').read_text())
    fault = any(r.get('state', '').startswith(('FAULT', 'STOPPED')) for r in samples)
    complete = all(v['reached'] for v in visits) and metadata.get('completion') == 'LANDED' and not fault
    nav = root / 'navigation.log'
    logged = 'Benchmark complete' in nav.read_text(errors='replace') if nav.exists() else None
    if logged is not None:
        complete = complete and logged
    with (directory / 'kinematics.csv').open() as f:
        kin = [r for r in csv.DictReader(f) if r['source'] == 'truth' and r['state'] == 'TRACKING']
    ts = np.array([r['stamp'] for r in tr])
    if len(tr) < 2:
        raise ValueError('Not enough physical truth samples')
    rb = Rotation.from_quat([r['q'] for r in tr]) * Rotation.from_euler(
        'ZYX', metadata['imu_orientation'], degrees=True).inv()
    body = np.array([r['p'] for r in tr]) - rb.apply(metadata['imu_position'])
    t = np.array([float(r['sim_time_s']) for r in kin])
    p = np.column_stack([np.interp(t, ts, body[:, i]) for i in range(3)]) if len(t) else np.empty((0, 3))
    speed = np.array([float(r['speed_mps']) for r in kin])
    dt = np.diff(t, append=t[-1]) if len(t) else np.array([])
    dt[(dt < 0) | (dt > .10001)] = 0
    zone_stats = []
    turn = visits[0]['stamp'] if visits[0]['reached'] else np.inf
    for direction, direction_mask in [('all', np.ones(len(t), bool)),
                                       ('outbound', t <= turn), ('return', t > turn)]:
        for z in recipe['zones']:
            mask = direction_mask & (p[:, 1] >= z['y_min']) & (p[:, 1] < z['y_max'])
            duration = float(dt[mask].sum())
            zone_stats.append(dict(zone=z['name'], direction=direction, time_s=duration,
                                   speed_mean_mps=float(np.dot(speed[mask], dt[mask]) / duration) if duration else None,
                                   speed_median_mps=float(np.median(speed[mask])) if mask.any() else None,
                                   speed_p95_mps=float(np.percentile(speed[mask], 95)) if mask.any() else None,
                                   speed_max_mps=float(speed[mask].max()) if mask.any() else None))
    lio_error = None
    if li:
        lt = np.array([r['stamp'] for r in li])
        inside = (lt >= ts[0]) & (lt <= ts[-1])
        physical = np.column_stack([np.interp(lt, ts, body[:, i]) for i in range(3)])
        errors = np.linalg.norm(np.array([r['p'] for r in li]) - physical, axis=1)[inside]
        lio_error = dict(max_m=float(errors.max()), rms_m=float(np.sqrt(np.mean(errors**2))),
                         nonpositive_stamp_steps=int(np.sum(np.diff(lt) <= 0)),
                         max_stamp_gap_s=float(np.diff(lt).max()) if len(lt) > 1 else None)
    contact_path = Path(contacts) if contacts else root / 'contacts.jsonl'
    cr, bad = rows(contact_path)
    cs = [r for r in cr if r.get('kind') == 'summary']
    ce = [r for r in cr if r.get('kind') == 'contact']
    contact = dict(measured=bool(cs), file=str(contact_path), malformed_lines=bad)
    if cs:
        last = cs[-1]
        valid = (bad == 0 and last['callback_errors'] == 0 and last['dropped_events'] == 0
                 and last['counts'].get('floor_found', 0) > 0
                 and last['counts'].get('floor_lost', 0) > 0 and last['sim_time_s'] >= ts[-1] - 1.)
        contact.update(valid=bool(valid), obstacle_reports=sum(r['category'] == 'obstacle' for r in ce),
                       other_external_reports=sum(r['category'] == 'other_external' for r in ce),
                       obstacle_impulse_reports=sum(r['category'] == 'obstacle' and r['max_impulse_norm'] > 1e-8 for r in ce),
                       callback_errors=last['callback_errors'], dropped_events=last['dropped_events'],
                       floor_counts=last['counts'])
        episodes = []
        for event in sorted((r for r in ce if r['category'] == 'obstacle' and r['max_impulse_norm'] > 1e-8),
                            key=lambda r: r['sim_time_s']):
            if not episodes or event['sim_time_s'] - episodes[-1]['last_sim_s'] > .25:
                episodes.append(dict(first_sim_s=event['sim_time_s'], last_sim_s=event['sim_time_s'],
                                     response_reports=0, max_impulse_norm=0.))
            episode = episodes[-1]
            episode['last_sim_s'] = event['sim_time_s']
            episode['response_reports'] += 1
            episode['max_impulse_norm'] = max(episode['max_impulse_norm'], event['max_impulse_norm'])
        contact['response_episodes_grouped_by_0_25s'] = episodes
    zones_traversed = all(any(r['zone'] == name and r['direction'] == direction and r['time_s'] > 0
                              for r in zone_stats)
                          for name in ['open_south', 'dense_center', 'open_north']
                          for direction in ['outbound', 'return'])
    config = root / 'loaded-common.yaml'
    out = dict(scenario='mixed', backend=metadata['backend'], mission_complete=bool(complete),
               benchmark_complete_log=logged, waypoint_radius_m=.8, stop_speed_mps=.7,
               waypoint_visits=visits, both_directions_traversed_all_zones=zones_traversed,
               last_controller_state=metadata.get('last_controller_state'),
               config_path=metadata['planner_config_path'],
               config_sha256=hashlib.sha256(config.read_bytes()).hexdigest() if config.exists() else None,
               map_sha256=recipe['generated_pcd_sha256'], malformed_telemetry_lines=malformed,
               tracking_sim_s=float(li[-1]['stamp'] - li[0]['stamp']) if li else 0.,
               actual_tracking_max=summary['sources'].get('truth', {}).get('tracking', {}),
               zone_speed=zone_stats, lio=lio_error, contacts=contact,
               ground_truth_usage='Offline body/zone diagnostics only, never planner or controller feedback',
               derivative_scope='About0.21s smoothed velocity derivatives; not raw contact shock maxima')
    out['contact_free_completion'] = (bool(complete and contact['obstacle_reports'] == 0
                                          and contact['other_external_reports'] == 0)
                                     if contact.get('valid') else None)
    if (root / 'operator-stop.json').exists():
        out['operator_stop'] = json.loads((root / 'operator-stop.json').read_text())
    for filename, key in [('hull-full-flight-clearance.json', 'full_flight_cad_clearance'),
                          ('exact-clearance.json', 'voxel_clearance')]:
        if (root / filename).exists():
            out[key] = json.loads((root / filename).read_text())
    (root / 'mixed-result.json').write_text(json.dumps(out, indent=2, ensure_ascii=False) + '\n')
    plot(root, assets, recipe, t, p, speed)
    chosen = [r for r in zone_stats if r['direction'] == 'all']
    zone_names = dict(open_south='남쪽 개방', transition_south='남쪽 연결',
                      dense_center='중앙 밀집', transition_north='북쪽 연결', open_north='북쪽 개방')
    table = ''.join('<tr><td>{}</td><td>{:.1f}</td><td>{}</td><td>{}</td></tr>'.format(
        zone_names[r['zone']], r['time_s'], '-' if r['speed_mean_mps'] is None else f'{r["speed_mean_mps"]:.2f}',
        '-' if r['speed_max_mps'] is None else f'{r["speed_max_mps"]:.2f}') for r in chosen)
    def embedded(path):
        return 'data:image/png;base64,' + base64.b64encode(path.read_bytes()).decode()
    contact_text = ('미계측' if not contact['measured'] else
                    '기록 유효성 확인 실패: 접촉 없음으로 판정하지 않음' if not contact.get('valid') else
                    f"기록 유효 · 장애물 보고 {contact['obstacle_reports']}건 · "
                    f"장애물 충격 보고 {contact['obstacle_impulse_reports']}건 · "
                    f"미분류 외부 보고 {contact['other_external_reports']}건")
    lio_text = ('자료 없음' if lio_error is None else
                f"최대 위치 오차 {lio_error['max_m']:.3f}m / RMS {lio_error['rms_m']:.3f}m · "
                f"stamp 역행 {lio_error['nonpositive_stamp_steps']}건")
    page = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>SUPER 혼합 맵 비행 결과</title>
<style>body{{font:16px system-ui;max-width:1300px;margin:32px auto;padding:0 20px;color:#172234}}img{{width:100%}}table{{border-collapse:collapse}}td,th{{padding:9px 18px;border-bottom:1px solid #ccc}}p{{line-height:1.7}}</style>
<h1>SUPER 혼합 맵 · Super-LIO · 공통 YAML</h1>
<p>임무 완주: <b>{'완료' if complete else '미완주'}</b> · 주행 {out['tracking_sim_s']:.2f} sim s<br>
원래 경유점 0.8m / 속도 0.7m/s 조건. 착륙만으로 완주를 판정하지 않습니다.<br>
접촉 계측: {html.escape(contact_text)}<br>LIO: {html.escape(lio_text)}</p>
<p>접촉 보고 수는 근접·유지·해제 이벤트를 포함하며 충돌 횟수와 다릅니다. 실제 충격 응답 구간과 기록 원문은 결과 JSON에 보존합니다.</p>
<img src="{embedded(root / 'mixed-flight.png')}" alt="혼합 맵과 실제 경로 및 구간별 속도">
<h2>실제 구간별 속도</h2><table><tr><th>구간</th><th>주행 시간 s</th><th>시간 가중 평균 m/s</th><th>최대 m/s</th></tr>{table}</table>
<p>구간 판정은 비행 후 실제 body 위치로만 계산합니다. 왕복·목표 부근 정지 시간도 포함하며, 영역 라벨이나 ground truth를 비행 제어에 사용하지 않습니다. 속도 차이는 이 맵의 해당 비행 관측값이며 모든 환경의 적응 성능을 보장하지 않습니다.</p>
<h2>실제 속도·가속도·저크와 최대값</h2><img src="{embedded(directory / 'motion.png')}" alt="속도 가속도 저크 최대값">
<p>가속도·저크는 약0.21초 평활 미분값입니다. nominal boundary는 실제 기체의 엄격한 제한을 뜻하지 않습니다. 위치 오차와 전체 원본 수치는 <a href="mixed-result.json">결과 JSON</a>에 보존합니다.</p></html>'''
    (root / 'mixed-report.html').write_text(page)
    print(json.dumps({k: out[k] for k in ['mission_complete', 'tracking_sim_s', 'actual_tracking_max', 'lio', 'contacts']}, indent=2))
    print(root / 'mixed-report.html')
    return out


def plot(root, assets, recipe, t, p, speed):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    cloud = read_pcd(assets / 'mixed_sparse_dense.pcd')
    obs = cloud[np.abs(cloud[:, 2]) >= 1e-7]
    xy = np.unique(np.floor(obs[:, :2] / .15).astype(int), axis=0) * .15 + .075
    fig, axes = plt.subplots(3, 1, figsize=(14, 8), gridspec_kw={'height_ratios': [1.25, 1, 1]})
    ax = axes[0]
    ax.scatter(xy[:, 1], xy[:, 0], s=.5, color='gray', alpha=.5)
    for z in recipe['zones']:
        ax.axvspan(z['y_min'], z['y_max'], color='#eeaa55' if z['name'] == 'dense_center' else '#65b8d0', alpha=.12)
    for y in [-50., 50.]:
        ax.add_patch(Circle((y, 0), 2.5, fill=False, edgecolor='green', ls='--'))
    if len(t):
        dots = ax.scatter(p[:, 1], p[:, 0], c=speed, cmap='turbo', s=3, vmin=0)
        fig.colorbar(dots, ax=ax, label='Body speed (m/s)', pad=.01)
    ax.set(xlim=(-55, 55), ylim=(-8.5, 8.5), xlabel='World y (m)', ylabel='World x (m)',
           title='OPEN → DENSE → OPEN, same YAML, round trip. Top view; altitude varies.')
    ax.set_aspect('equal', adjustable='box')
    elapsed = t - t[0] if len(t) else t
    for ax, values, label in [(axes[1], speed, 'Body speed (m/s)'),
                              (axes[2], p[:, 2], 'Body altitude (m)')]:
        ax.plot(elapsed, values, lw=1)
        if len(t):
            dense = (p[:, 1] >= -18) & (p[:, 1] < 18)
            ax.fill_between(elapsed, 0, 1, where=dense, transform=ax.get_xaxis_transform(),
                            color='#eeaa55', alpha=.2, label='Dense-center residence')
            if label.startswith('Body speed'):
                i = int(np.argmax(speed))
                ax.scatter(elapsed[i], speed[i], c='red')
                ax.text(.02, .9, f'MAX {speed[i]:.3f} m/s', transform=ax.transAxes, color='darkred')
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
        ax.legend(loc='upper right', fontsize=8)
    axes[-1].set_xlabel('Simulation seconds since TRACKING began')
    fig.tight_layout()
    fig.savefig(root / 'mixed-flight.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='Flight directory containing metadata.json / kinematics.csv')
    parser.add_argument('--contacts', type=Path)
    args = parser.parse_args()
    analyze(args.directory, args.contacts)
