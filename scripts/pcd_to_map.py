#!/usr/bin/env python3
"""把 LIO-SAM 保存的 PCD 点云地图转成 nav2_map_server 能读的 PGM + YAML。

用法：
    python3 scripts/pcd_to_map.py --pcd ~/pnc_maps/lio_room/CornerMap.pcd \
        --pcd ~/pnc_maps/lio_room/SurfMap.pcd --out ~/pnc_maps/lio_room/map \
        --resolution 0.05 --z-min 0.15 --z-max 1.5

说明：
- 同时支持 PCD 的 `DATA binary` 和 `DATA ascii`（LIO-SAM 用 savePCDFileBinary）
- 高度带（相对地面）内的点投影成障碍；地面高度用低分位点估计，避免用绝对 0
- 输出 <out>.pgm（0=占用 205=未知 254=自由）+ <out>.yaml（带 origin / resolution）

后续：
    ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py map:=<out>.yaml
"""

import argparse
import os
import sys

import numpy as np


def read_pcd(path):
    """读 PCD（ascii / binary），返回 Nx3 的 xyz（float32）。"""
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError(f'{path}: PCD 头不完整')
            text = line.decode('ascii', errors='ignore').strip()
            if not text or text.startswith('#'):
                continue
            key, _, value = text.partition(' ')
            header[key.upper()] = value.strip()
            if key.upper() == 'DATA':
                break
        data_kind = header['DATA'].lower()
        fields = header.get('FIELDS', 'x y z').split()
        sizes = [int(v) for v in header.get('SIZE', '4 4 4').split()]
        types = header.get('TYPE', 'F F F').split()
        counts = [int(v) for v in header.get('COUNT', ' '.join(['1'] * len(fields))).split()]
        n_points = int(header.get('POINTS', 0))
        if n_points == 0:
            n_points = int(header.get('WIDTH', 0)) * int(header.get('HEIGHT', 1))

        if data_kind == 'ascii':
            raw = np.loadtxt(f, dtype=np.float64, ndmin=2)
            idx = {name: i for i, name in enumerate(fields)}
            xyz = np.stack([raw[:, idx['x']], raw[:, idx['y']], raw[:, idx['z']]], axis=1)
            return xyz.astype(np.float32)

        if data_kind != 'binary':
            raise RuntimeError(f'{path}: 不支持 DATA {data_kind}（binary_compressed 请先 pcl_convert_pcd_ascii_binary）')

        # binary：按字段布局拼 numpy dtype
        np_types = {('F', 4): '<f4', ('F', 8): '<f8', ('U', 1): '<u1', ('U', 2): '<u2',
                    ('U', 4): '<u4', ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4'}
        names, formats, offsets, offset = [], [], [], 0
        for name, size, ftype, count in zip(fields, sizes, types, counts):
            key = (ftype.upper(), size)
            if key not in np_types:
                raise RuntimeError(f'{path}: 不支持的字段类型 {name} {ftype}{size}')
            names.append(name)
            formats.append((np_types[key], (count,)) if count > 1 else np_types[key])
            offsets.append(offset)
            offset += size * count
        blob = f.read(n_points * offset)
        arr = np.frombuffer(blob, dtype=np.dtype({
            'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': offset}))
        return np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', action='append', required=True, help='输入 PCD（可多次指定）')
    ap.add_argument('--out', required=True, help='输出前缀（生成 <out>.pgm / <out>.yaml）')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=0.15, help='相对地面高度的下限 (m)')
    ap.add_argument('--z-max', type=float, default=1.5, help='相对地面高度的上限 (m)')
    ap.add_argument('--pad', type=float, default=0.5, help='地图外扩边距 (m)')
    ap.add_argument('--dilate', type=int, default=1, help='障碍膨胀格数')
    args = ap.parse_args()

    pts = []
    for p in args.pcd:
        if not os.path.exists(p):
            print(f'跳过不存在的 {p}')
            continue
        xyz = read_pcd(p)
        print(f'{p}: {len(xyz)} 点')
        pts.append(xyz)
    if not pts:
        print('没有可用的点云')
        return 1
    xyz = np.concatenate(pts, axis=0)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) == 0:
        print('点云为空')
        return 1

    # 地面高度：取最低 5% 分位的 z 作为地面（点云里地面点最多）
    floor = float(np.percentile(xyz[:, 2], 5))
    print(f'地形点数 {len(xyz)}，地面 z≈{floor:.3f}')

    z_rel = xyz[:, 2] - floor
    keep = (z_rel >= args.z_min) & (z_rel <= args.z_max)
    obs = xyz[keep]
    print(f'高度带 [{args.z_min}, {args.z_max}] m 内障碍点 {len(obs)}（{100.0 * len(obs) / len(xyz):.1f}%）')

    res = args.resolution
    xmin, ymin = xyz[:, 0].min() - args.pad, xyz[:, 1].min() - args.pad
    xmax, ymax = xyz[:, 0].max() + args.pad, xyz[:, 1].max() + args.pad
    width = max(1, int(np.ceil((xmax - xmin) / res)))
    height = max(1, int(np.ceil((ymax - ymin) / res)))
    print(f'栅格 {width} x {height} @ {res} m，origin=({xmin:.2f}, {ymin:.2f})')

    grid = np.full((height, width), 254, dtype=np.uint8)   # 254 = free
    ix = ((obs[:, 0] - xmin) / res).astype(np.int32)
    iy = ((obs[:, 1] - ymin) / res).astype(np.int32)
    ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    grid[iy[ok], ix[ok]] = 0                               # 0 = occupied

    # 障碍膨胀（防止规划贴边）
    for _ in range(max(0, args.dilate)):
        occ = grid == 0
        grown = occ.copy()
        grown[1:, :] |= occ[:-1, :]
        grown[:-1, :] |= occ[1:, :]
        grown[:, 1:] |= occ[:, :-1]
        grown[:, :-1] |= occ[:, 1:]
        grid[grown] = 0

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = args.out + '.pgm'
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        f.write(np.flipud(grid).tobytes())                 # PGM 行序：上到下

    yaml_path = args.out + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write('image: ' + os.path.basename(pgm_path) + '\n')
        f.write(f'resolution: {res}\n')
        f.write(f'origin: [{xmin:.4f}, {ymin:.4f}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')

    occ_cells = int((grid == 0).sum())
    print(f'写出 {pgm_path}（占用格 {occ_cells}，{100.0 * occ_cells / grid.size:.1f}%）')
    print(f'写出 {yaml_path}')
    print(f'\n用法：ros2 launch pnc_nav_bringup gz_sim_3d_lidar_bringup.launch.py map:={yaml_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
