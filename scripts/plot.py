import glob
import numpy as np
from matplotlib import pyplot as plt
import open3d as o3d

import sys
sys.path.append('/home/jamiecho/Repos/Ravel/PhoneBot/pyphonebot/')
from phonebot.vis.viewer import ProxyViewer
from phonebot.vis.viewer.proxy_command import ProxyCommand
from phonebot.vis.viewer.proxy_commands import AddLineStripCommand, AddGridCommand

import pyqtgraph as pg
import pyqtgraph.opengl as gl

import time


class AddPointsCommand(ProxyCommand):
    def __init__(self, name='points'):
        super().__init__(name)

    def __call__(self, viewer: ProxyViewer):
        item = gl.GLScatterPlotItem()
        item.setData(pos=np.empty((0, 3), dtype=np.float32))
        item.setGLOptions('opaque')
        viewer.items_[self.name] = item
        viewer.handlers_[self.name] = item.setData
        viewer.widget_.addItem(item)


def main():
    data_queue, event_queue, command_queue = ProxyViewer.create()
    command_queue.put(AddPointsCommand(name='points'))
    command_queue.put(AddLineStripCommand(name='view'))
    command_queue.put(AddGridCommand(name='grid'))

    pose_files = sorted(glob.glob('/tmp/pose_*.txt'))
    cloud_files = sorted(glob.glob('/tmp/view_*.ply'))
    poses = []
    clouds = []
    for pose_file, cloud_file in zip(pose_files, cloud_files):
        view = np.loadtxt(pose_file)
        cloud = np.asarray(o3d.io.read_point_cloud(cloud_file).points)
        # cloud = o3d.io.read_point_cloud(cloud_file)
        poses.append(view)
        clouds.append(cloud)
    poses = np.asarray(poses)

    index = 0
    while True:
        imin = max(0, index-16)
        imax = min(index, len(poses))
        trajectory = poses[imin:imax][..., :3, 3]
        # print(index, imin, imax)
        # print('pose', poses[index])
        # print('traj', trajectory[-1])
        data_queue.put(
            dict(points=dict(pos=clouds[index]), view=dict(pos=trajectory)))
        index = (index + 1) % len(poses)
        time.sleep(0.001)


if __name__ == '__main__':
    main()
