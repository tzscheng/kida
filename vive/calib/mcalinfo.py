#!/usr/bin/env python3
"""저장한 .mcal 이 멀쩡한 프로필인지 확인합니다.

    ./calib/mcalinfo.py ~/Documents/manus-calibrations/Calibration.mcal
    ./calib/mcalinfo.py calib/p*/*.mcal          # 기존 것과 나란히 비교

형식: ~<hex 제목>~ + 헤더 몇 바이트 + gzip(JSON).
손가락별 proportions / fingerCurve / minMaxRange 등이 들어 있고, 값이 전부
0이거나 키가 비어 있으면 캘리브레이션이 제대로 안 실린 것입니다.
"""
import sys, gzip, json


def load(path):
    raw = open(path, 'rb').read()
    if not raw.startswith(b'~'):
        raise ValueError('~ 헤더가 없습니다 (.mcal 아님?)')
    title = bytes.fromhex(raw[1:raw.index(b'~', 1)].decode()).decode()
    i = raw.find(b'\x1f\x8b\x08')
    if i < 0:
        raise ValueError('gzip 스트림을 못 찾았습니다')
    return title, json.loads(gzip.decompress(raw[i:]))


def main(paths):
    for p in paths:
        print('=' * 60)
        print(p)
        try:
            title, j = load(p)
        except Exception as e:
            print('  읽기 실패:', e)
            continue
        print('  제목    :', title)
        print('  손가락  :', ', '.join(j) if isinstance(j, dict) else type(j).__name__)
        if not isinstance(j, dict):
            continue
        for finger, d in j.items():
            if not isinstance(d, dict):
                continue
            # 값이 실제로 채워졌는지: 0이 아닌 실수의 개수를 센다
            n = nz = 0
            def walk(o):
                nonlocal n, nz
                if isinstance(o, dict):
                    for v in o.values(): walk(v)
                elif isinstance(o, list):
                    for v in o: walk(v)
                elif isinstance(o, (int, float)) and not isinstance(o, bool):
                    n += 1
                    if abs(o) > 1e-9: nz += 1
            walk(d)
            flag = '' if nz else '   <-- 전부 0, 의심'
            print('    %-8s 키 %2d개, 수치 %4d개 중 0 아닌 값 %4d개%s'
                  % (finger, len(d), n, nz, flag))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(2)
    main(sys.argv[1:])
