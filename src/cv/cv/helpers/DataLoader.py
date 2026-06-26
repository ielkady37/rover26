#!/usr/bin/env python3
"""Image/video data loader for offline YOLOPv2 inference."""

import glob
import os
from pathlib import Path

import cv2
import numpy as np

from cv.helpers.CVUtilities import CVUtilities


class LoadImages:
    """Iterate over images, videos, or a live camera stream for inference.

    Yields ``(path, img, img0, cap)`` tuples where ``img`` is the
    letterboxed/normalised tensor-ready array and ``img0`` is the original.
    """

    def __init__(self, path, img_size=640, stride=32):
        self.img_size = img_size
        self.stride = stride

        stream_source = None
        if isinstance(path, (int, np.integer)):
            stream_source = str(path)
        else:
            path_str = str(path)
            if path_str.isdigit() or path_str.startswith('/dev/video'):
                stream_source = path_str

        if stream_source is not None:
            self.files = [stream_source]
            self.nf = 1
            self.video_flag = [True]
            self.mode = 'stream'
            self.new_video(stream_source)
            if not self.cap.isOpened():
                raise Exception(f'ERROR: failed to open stream {stream_source}')
            return

        p = str(Path(path).absolute())
        if '*' in p:
            files = sorted(glob.glob(p, recursive=True))
        elif os.path.isdir(p):
            files = sorted(glob.glob(os.path.join(p, '*.*')))
        elif os.path.isfile(p):
            files = [p]
        else:
            raise Exception(f'ERROR: {p} does not exist')

        img_formats = ['bmp', 'jpg', 'jpeg', 'png', 'tif', 'tiff', 'dng', 'webp', 'mpo']
        vid_formats = ['mov', 'avi', 'mp4', 'mpg', 'mpeg', 'm4v', 'wmv', 'mkv']
        images = [x for x in files if x.split('.')[-1].lower() in img_formats]
        videos = [x for x in files if x.split('.')[-1].lower() in vid_formats]
        ni, nv = len(images), len(videos)

        self.files = images + videos
        self.nf = ni + nv
        self.video_flag = [False] * ni + [True] * nv
        self.mode = 'image'
        if any(videos):
            self.new_video(videos[0])
        else:
            self.cap = None
        assert self.nf > 0, (
            f'No images or videos found in {p}. '
            f'Supported formats:\nimages: {img_formats}\nvideos: {vid_formats}'
        )

    def __iter__(self):
        self.count = 0
        return self

    def __next__(self):
        if self.count == self.nf:
            raise StopIteration
        path = self.files[self.count]

        if self.video_flag[self.count]:
            self.mode = 'video'
            ret_val, img0 = self.cap.read()
            if not ret_val:
                self.count += 1
                self.cap.release()
                if self.count == self.nf:
                    raise StopIteration
                path = self.files[self.count]
                self.new_video(path)
                ret_val, img0 = self.cap.read()
            self.frame += 1
            print(f'video {self.count + 1}/{self.nf} ({self.frame}/{self.nframes}) {path}: ', end='')
        else:
            self.count += 1
            img0 = cv2.imread(path)
            assert img0 is not None, 'Image Not Found ' + path

        img0 = cv2.resize(img0, (1280, 720), interpolation=cv2.INTER_LINEAR)
        img = CVUtilities.letterbox(img0, self.img_size, stride=self.stride)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)

        return path, img, img0, self.cap

    def new_video(self, path):
        """Open a new video file and reset frame counter."""
        self.frame = 0
        self.cap = cv2.VideoCapture(path)
        self.nframes = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

    def __len__(self):
        return self.nf
